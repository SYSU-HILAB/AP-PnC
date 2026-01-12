# ROS2 Debug Visualization Node for feature/lu_mpc

## Overview

Add a loosely coupled ROS2 visualization node for post-hoc analysis of simulation data from CSV logs. The visualization will show:
- Reference trajectory (from CSV)
- Odometry evolution over time (position, velocity, attitude)
- Full aerodynamics breakdown (forces, moments, coefficients, angles, velocities in both FLU and FRD frames)

## Architecture: Loose Coupling Principles

The design follows these separation of concerns layers:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Layer 1: Data Access (io/)                        │
│  - CSVReader: Parse CSV files → TrajectoryData                      │
│  - No ROS2 dependencies, pure data processing                        │
│  - Testable without ROS2 runtime                                     │
├─────────────────────────────────────────────────────────────────────┤
│                  Layer 2: Markers (markers/)                         │
│  - MarkerFactory: Create RViz markers (lines, arrows, axes)         │
│  - Reusable across publishers, no data source knowledge             │
├─────────────────────────────────────────────────────────────────────┤
│                 Layer 3: Publishers (publishers/)                    │
│  - TrajectoryPublisher, StatePublisher, AeroPublisher               │
│  - Convert TrajectoryData → ROS2 messages + markers                 │
│  - Stateless transformations, no business logic                     │
├─────────────────────────────────────────────────────────────────────┤
│                   Layer 4: Node (nodes/)                             │
│  - DebugVizNode: Orchestrate publishers, manage ROS2 lifecycle      │
│  - Handle timing/playback, minimal state management                 │
└─────────────────────────────────────────────────────────────────────┘
```

## Directory Structure

```
feature/lu_mpc/
├── claude_plans/
│   └── ros2_debug_viz.md          # This plan
├── ros2_viz/                      # New ROS2 visualization module
│   ├── __init__.py
│   ├── __main__.py                # Entry point for direct execution
│   ├── config/
│   │   ├── __init__.py
│   │   └── viz_config.py          # VisualizationConfig dataclass
│   ├── io/
│   │   ├── __init__.py
│   │   └── csv_reader.py          # CSVReader, TrajectoryData
│   ├── markers/
│   │   ├── __init__.py
│   │   └── marker_factory.py      # Marker creation utilities
│   ├── publishers/
│   │   ├── __init__.py
│   │   ├── trajectory_pub.py      # Trajectory markers
│   │   ├── state_pub.py           # State viz (velocity, attitude)
│   │   └── aero_pub.py            # Aero forces/moments/angles
│   └── nodes/
│       ├── __init__.py
│       └── debug_viz_node.py      # Main ROS2 node
├── launch/
│   └── lu_mpc_viz.launch.py       # ROS2 launch file
└── rviz/
    └── lu_mpc_debug.rviz          # RViz configuration
```

## Implementation Plan

### Step 1: Configuration Layer

**File**: `feature/lu_mpc/ros2_viz/config/viz_config.py`

Create `VisualizationConfig` dataclass following existing patterns:
```python
@dataclasses.dataclass
class VisualizationConfig:
    # File paths
    reference_trajectory_csv: str = "model/refs/planning.csv"
    simulation_results_csv: str = ""

    # ROS2 topics
    node_name: str = "lu_mpc_debug_viz"
    marker_array_topic: str = "/lu_mpc/marker_array"

    # Visualization options
    publish_rate_hz: float = 10.0
    downsample_factor: int = 1

    # Frame IDs
    world_frame_id: str = "world"
    body_frame_id: str = "base_link"

    # Marker styles (RGB colors, scales, widths)
    ...
```

### Step 2: Data Access Layer

**File**: `feature/lu_mpc/ros2_viz/io/csv_reader.py`

Create `TrajectoryData` dataclass and `CSVReader`:
```python
@dataclasses.dataclass
class TrajectoryData:
    times: np.ndarray
    positions: np.ndarray          # N x 3
    velocities: np.ndarray          # N x 3
    angular_rates: np.ndarray       # N x 3
    quaternions: np.ndarray         # N x 4 (w,x,y,z)
    motor_thrusts: np.ndarray       # N x 4

    # Aerodynamic data (optional)
    aero_forces_flu: np.ndarray = None   # N x 3
    aero_forces_frd: np.ndarray = None   # N x 3
    aero_moments_flu: np.ndarray = None  # N x 3
    aero_moments_frd: np.ndarray = None  # N x 3
    alpha: np.ndarray = None             # N
    beta: np.ndarray = None              # N
    velocity_flu: np.ndarray = None      # N x 3
    velocity_frd: np.ndarray = None      # N x 3

class CSVReader:
    def read_reference_trajectory(self, csv_path: str) -> TrajectoryData
    def read_simulation_results(self, csv_path: str) -> TrajectoryData
```

### Step 3: Marker Factory

**File**: `feature/lu_mpc/ros2_viz/markers/marker_factory.py`

Create reusable marker creation methods:
```python
class MarkerFactory:
    @staticmethod
    def create_trajectory_line(positions, color, frame_id, ns) -> Marker

    @staticmethod
    def create_vector_arrow(position, vector, color, scale, frame_id, ns) -> Marker

    @staticmethod
    def create_attitude_axes(position, quaternion, frame_id, ns) -> List[Marker]
```

### Step 4: Publishers

**Files**:
- `trajectory_pub.py`: Trajectory line markers (reference + simulation)
- `state_pub.py`: Velocity vectors, attitude axes
- `aero_pub.py`: Force/moment vectors, alpha/beta indicators, multi-frame velocities

### Step 5: Main Node

**File**: `feature/lu_mpc/ros2_viz/nodes/debug_viz_node.py`

Orchestrate publishers with timer-based playback:
```python
class DebugVizNode(rclpy.node.Node):
    def __init__(self, config: VisualizationConfig):
        # Load CSV data
        # Create publishers
        # Setup timer for playback

    def publish_all_data(self):
        # Publish markers for current timestep
```

### Step 6: CSV Export from Simulation

**Modify**: `feature/lu_mpc/model/sim/sim_gd_model.py`

Add CSV export method to save complete aerodynamic data:
```python
def save_simulation_csv(results: dict, csv_path: str, include_aero_data: bool = True):
    """Save simulation results to CSV with full aerodynamic data."""
```

### Step 7: Justfile Integration

**Add to justfile**:
```bash
# Visualize CSV data in RViz
# Usage: just viz-lu-mpc [ref_csv] [sim_csv]
viz-lu-mpc ref_csv="" sim_csv="":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -m feature.lu_mpc.ros2_viz \
        --ref-csv "{{ref_csv}}" --sim-csv "{{sim_csv}}"

# Simulate and visualize in one command
viz-sim-and-viz sim_time="2.0":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "
    from feature.lu_mpc.model.sim.sim_gd_model import create_gd_simulator, GdSimConfig, save_simulation_csv
    ...
    "
    just viz-lu-mpc "$AP_PNC_DIR/artifacts/simulations/temp_data/sim_results.csv"
```

## Critical Files to Create

1. `feature/lu_mpc/ros2_viz/config/viz_config.py` - Configuration
2. `feature/lu_mpc/ros2_viz/io/csv_reader.py` - CSV parsing
3. `feature/lu_mpc/ros2_viz/markers/marker_factory.py` - Marker utilities
4. `feature/lu_mpc/ros2_viz/publishers/aero_pub.py` - Aero visualization
5. `feature/lu_mpc/ros2_viz/nodes/debug_viz_node.py` - Main node

## Files to Modify

1. `feature/lu_mpc/model/sim/sim_gd_model.py` - Add CSV export with full aero data
2. `justfile` - Add viz commands

## Verification

1. Run simulation with aero data collection:
   ```bash
   just sim-viz-gd 2.0 true
   ```

2. Save to CSV (new method):
   ```python
   save_simulation_csv(results, "artifacts/simulations/temp_data/sim_results.csv", include_aero_data=True)
   ```

3. Launch visualization:
   ```bash
   just viz-lu-mpc "artifacts/simulations/temp_data/sim_results.csv"
   ```

4. Verify in RViz:
   - Green trajectory line (reference)
   - Blue trajectory line (simulation)
   - Orange velocity vectors
   - Red force vectors (aerodynamic)
   - Magenta moment vectors
   - RGB attitude axes
   - Proper frame transforms

## Design Principles Followed

1. **Configuration-driven**: All parameters via `@dataclass` with defaults
2. **Layered separation**: io/ → markers/ → publishers/ → nodes/
3. **No coupling in data layer**: CSVReader has no ROS2 imports
4. **Reusable markers**: MarkerFactory works for any publisher
5. **Stateless publishers**: Pure transformations, no internal state
6. **Factory functions**: Following existing pattern (`create_visualizer()`)
7. **Consistent justfile**: Use `$AP_PNC_DIR/.acados_venv/bin/python`
