# Error-State MPC Trajectory Tracking Visualization

The `viz-traj-tracking-elmpc` command visualizes closed-loop trajectory tracking using the error-state linear MPC (elMPC) controller. This is an alternative to the nonlinear MPC (NMPC) tracker, using linearized error dynamics for faster computation.

## What is Error-State MPC?

Error-State MPC (elMPC) is a linear Model Predictive Control approach that:

- **Uses error dynamics**: Instead of controlling absolute states, it minimizes the error between current state and reference trajectory
- **Linearized model**: Linearizes dynamics around the reference trajectory for faster QP solving
- **9D error state**: `[err_p(3), err_v(3), err_theta(3)]` - position, velocity, and attitude errors
- **4D control**: `[increment_aT(1), increment_omega(3)]` - increments to thrust and angular velocity
- **Time-varying parameters**: 28D parameter vector from reference trajectory includes velocity, rotation, thrust, angular rate, and aerodynamic forces

**Key differences from NMPC**:
| Feature | NMPC | elMPC |
|---------|------|-------|
| State dimension | 18D (full state) | 9D (error state) |
| Dynamics | Nonlinear | Linearized around reference |
| Solver type | SQP (nonlinear NLP) | QP (quadratic programming) |
| Compute time | Slower | Faster |
| Tracking precision | Global optimum | Local (around reference) |

## Quick Start

```bash
# Source ROS2 first
source /opt/ros/humble/setup.bash

# Run with default settings (hover mode)
just viz-traj-tracking-elmpc

# Run with custom trajectory
just viz-traj-tracking-elmpc --trajectory /path/to/trajectory.csv

# Run with custom RViz config
just viz-traj-tracking-elmpc --rviz-config path/to/config.rviz
```

## Command Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `--trajectory` | Path | `artifacts/simulations/temp_data.csv` | Path to trajectory CSV file |
| `--real-time-factor` | float | `1.0` | Speed multiplier (1.0 = real-time, 2.0 = 2x speed) |
| `--hover` | flag | `true` (enabled by default) | Run in hover mode (fixed reference) vs tracking mode |
| `--disable-aero` | flag | `false` | Skip publishing aerodynamic data |
| `--rviz-config` | Path | `feature/lu_mpc/ros/config/nmpc_viz.rviz` | Custom RViz2 configuration |
| `--no-rviz` | flag | `false` | Don't launch RViz2 (run simulation only) |

### Mode Selection

The `--hover` flag controls the controller behavior:

**Hover mode** (`--hover`, default):
- Controller maintains position at the initial reference point
- Reference trajectory is not advanced
- Useful for testing stability and hover performance

**Tracking mode** (no `--hover` flag):
- Controller follows the full trajectory from the CSV
- Reference advances at each control step
- Useful for evaluating trajectory tracking performance

To run in tracking mode, modify the justfile recipe to remove the `--hover` flag, or run the Python script directly:

```bash
./pyp test/lu_mpc/ros/user_test_traj_tracking_elmpc_viz.py --rviz-config feature/lu_mpc/ros/config/nmpc_viz.rviz
```

## Architecture

### Components

```
user_test_traj_tracking_elmpc_viz.py
    │
    ├──> MpcOrchestrator (ROS2 node)
    │       │
    │       ├──> ErrStateLmpcAngVelActuatedController
    │       │       │
    │       │       ├──> ACADOS solver (error-state OCP)
    │       │       ├──> PD controller (thrust tracking)
    │       │       ├──> PID controller (angular velocity)
    │       │       └── QP mixer (motor allocation)
    │       │
    │       └──> GdModelSimulator (CasADi dynamics)
    │
    └──> NmpcVisualizer (RViz2 publisher)
            │
            ├──> /nmpc/odometry
            ├──> /nmpc/reference_path
            ├──> /nmpc/tracking_path
            └──> /nmpc/motor_thrusts
```

### Controller Flow

1. **State estimation**: Get current state from simulator
2. **Error computation**: Compute error state `[err_p, err_v, err_theta]` between current and reference
3. **Parameter update**: Update time-varying parameters (28D) from reference window
4. **OCP solve**: ACADOS solves the linear MPC problem
5. **Control extraction**: Get control increments `[increment_aT, increment_omega]`
6. **Sub-controllers**:
   - PD controller converts `aT_cmd` to total thrust
   - PID controller converts `omega_cmd` to torque
   - QP mixer maps `[thrust, torque]` to motor thrusts
7. **Simulation step**: Apply controls to dynamics model

## Configuration

The elMPC controller uses two configuration classes:

### 1. ErrorStateLinearMpcConfig (`feature/lu_mpc/controller/config/ocp_err_state_cfg.py`)

Defines MPC solver parameters:

```python
@dataclass
class ErrorStateLinearMpcConfig:
    # Cost weights
    q_pos: np.ndarray      # Position error weights [x, y, z]
    q_vel: np.ndarray      # Velocity error weights [vx, vy, vz]
    q_theta: np.ndarray    # Angular error weights [phi, theta, psi]
    r_u: np.ndarray        # Control effort weights

    # Control increments bounds
    lb_u: np.ndarray       # Lower bounds
    ub_u: np.ndarray       # Upper bounds

    # Timing
    ctrl_dt: float = 0.1   # Control timestep
    nodes: int = 5         # Prediction horizon nodes
```

**Preset configurations**:
- `create_default()`: Standard tracking weights
- `create_high_gain()`: Higher gains for tighter tracking

### 2. ErrStateLmpcAngVelActuatedCfg (`feature/lu_mpc/controller/config/ctrl_err_state_lmpc_cfg.py`)

Combines MPC config with sub-controller configs:

```python
@dataclass
class ErrStateLmpcAngVelActuatedCfg:
    mpc_config: ErrorStateLinearMpcConfig
    trajectory: TrajectoryReference
    pd_config: PDControllerConfig
    pid_config: PIDAntiWindupConfig
    qp_mixer_config: QpMixerConfig
    json_path: Path  # ACADOS solver cache
    is_hover_task: bool
```

## Trajectory CSV Format

The trajectory CSV must contain the following columns:

```csv
time,position_x,position_y,position_z,velocity_x,velocity_y,velocity_z,quat_quad_flu_w,quat_quad_flu_x,quat_quad_flu_y,quat_quad_flu_z,...
```

Required columns:
- `time`: Timestamp (seconds)
- `position_*`: Position in world frame (m)
- `velocity_*`: Velocity in world frame (m/s)
- `quat_quad_flu_*`: Quaternion in FLU frame [w, x, y, z]

See `feature/lu_mpc/controller/trajectory_loader.py` for full format specification.

## ROS Topics

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/nmpc/odometry` | `nav_msgs/Odometry` | Vehicle position, velocity, orientation from simulator |
| `/nmpc/reference_path` | `nav_msgs/Path` | Full reference trajectory (static) |
| `/nmpc/tracking_path` | `nav_msgs/Path` | Actual tracking path (rolling buffer) |
| `/nmpc/motor_thrusts` | `std_msgs/Float32MultiArray` | Motor thrust commands [N] |

## Troubleshooting

### "ACADOS library not found"

The `just viz-traj-tracking-elmpc` recipe automatically sets `LD_LIBRARY_PATH`. If running the Python script directly:

```bash
export LD_LIBRARY_PATH=/home/rec/AP-PnC/mpc_framework/lib:$LD_LIBRARY_PATH
./pyp test/lu_mpc/ros/user_test_traj_tracking_elmpc_viz.py
```

### "Solver failed with status X"

The ACADOS solver may fail if:
- Initial error state is too large (trajectory starts far from hover)
- Control increments are saturated
- Prediction horizon is too short

Try:
1. Reducing `real_time_factor` to give solver more time
2. Increasing `nodes` in `ErrorStateLinearMpcConfig` for longer horizon
3. Checking that initial state matches trajectory start

### "NaN detected in simulation state"

The simulation has diverged. Common causes:
- Control gains too high (try `create_default()` instead of `create_high_gain()`)
- Integration timestep too large
- Reference trajectory contains invalid values

### RViz2 shows nothing

1. Check that `srunall` is running (or launch manually)
2. Verify ROS2 is sourced: `source /opt/ros/humble/setup.bash`
3. Check RViz2 configuration has correct frame IDs
4. Use `ros2 topic list` to verify topics are publishing

### Trajectory not found

The default trajectory is `artifacts/simulations/temp_data.csv`. Generate one with:

```bash
# Run the trajectory planner
./scripts/tools/plan
```

Or specify a custom path:

```bash
just viz-traj-tracking-elmpc --trajectory /path/to/your/trajectory.csv
```

## Performance Tuning

### Controller Gains

Edit `feature/lu_mpc/controller/config/ocp_err_state_cfg.py:72`:

```python
@classmethod
def create_high_gain(cls) -> "ErrorStateLinearMpcConfig":
    return cls(
        q_pos=np.array([20.0, 20.0, 150.0]),  # Increase for tighter tracking
        q_vel=np.array([2.0, 2.0, 2.0]),
        q_theta=np.array([70.0, 70.0, 180.0]),
    )
```

**Guidelines**:
- Higher `q_pos` → tighter position tracking, more aggressive
- Higher `q_theta` → better attitude regulation
- Lower `r_u` → allow larger control increments (more aggressive)

### Horizon Length

Edit `feature/lu_mpc/controller/config/ocp_err_state_cfg.py:55`:

```python
ctrl_dt: float = 0.1   # Control timestep (smaller = more frequent updates)
nodes: int = 5         # Prediction nodes (larger = longer horizon, slower)
```

**Trade-offs**:
- Longer horizon (`nodes` ↑): Better prediction, slower solve time
- Shorter timestep (`ctrl_dt` ↓): More frequent control, higher CPU load

## See Also

- **[Quick Start Guide](quickstart.md)** - General installation and setup
- **[Control Stack & Module Reference](control-stack.md)** - Full ROS2 architecture
- **[ACADOS Documentation](https://github.com/acados/acados)** - Solver details
- **`feature/lu_mpc/controller/ctrl_err_state_lmpc_ang_vel_actuated.py`** - Controller implementation
