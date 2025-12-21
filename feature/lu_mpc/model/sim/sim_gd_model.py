# -*- coding: future_fstrings -*-
#
# Ground-Truth Aerodynamics Model Simulation with ACADOS
# Created on 2024-12-22
#
# This module creates a high-fidelity simulation environment that combines
# ACADOS integration capabilities with the ground-truth GD aerodynamics model.
# Based on the tailsitter_flu() model structure with SymbolicGdModel integration.
#

import casadi as cs
import numpy as np
import dataclasses
from typing import Tuple, Optional, List
from acados_template import AcadosModel, AcadosSim, AcadosSimSolver
from .gd_model_casadi import SymbolicGdModel, GdModelConfig, DEFAULT_CONFIG


@dataclasses.dataclass
class GdSimConfig:
    """Configuration for the GD model simulation."""
    # Simulation parameters
    dt: float = 0.01  # Time step in seconds
    sim_time: float = 5.0  # Total simulation time in seconds

    # ACADOS solver options
    integrator_type: str = 'IRK'  # 'IRK', 'ERK', 'GNSF'
    num_stages: int = 3  # Number of stages for implicit integrator
    num_steps: int = 1  # Number of steps per integration interval
    newton_iter: int = 3  # Newton iterations for implicit solver

    # Aircraft physical parameters (from tailsitter_flu)
    mass: float = 1.0  # kg
    J: np.ndarray = None  # Inertia matrix
    c_tau: float = 0.013  # Torque coefficient
    dx: float = 0.08838834764831845  # Arm length x
    dy: float = 0.08838834764831845  # Arm length y

    # Frame transformation quaternions
    q_flu_to_frd: np.ndarray = None  # FLU to FRD transformation
    q_ned_to_world: np.ndarray = None  # NED to world transformation

    # Aerodynamic model configuration
    aero_config: GdModelConfig = DEFAULT_CONFIG

    def __post_init__(self):
        """Initialize default values."""
        if self.J is None:
            self.J = np.array([0.01715066, 0.01818037, 0.01987597])
        if self.q_flu_to_frd is None:
            # FLU to FRD transformation (0, 1, 0, 0)
            self.q_flu_to_frd = np.array([0.0, 1.0, 0.0, 0.0])
        if self.q_ned_to_world is None:
            # NED to local-world transformation (0, 0.707, 0, 0.707)
            self.q_ned_to_world = np.array([0.0, 0.707, 0.0, 0.707])


# Default configuration instance
DEFAULT_SIM_CONFIG = GdSimConfig()


class GdModelSimulator:
    """High-fidelity tailsitter simulator using GD aerodynamics model."""

    def __init__(self, config: GdSimConfig = DEFAULT_SIM_CONFIG):
        """
        Initialize the GD model simulator.

        Args:
            config: Simulation configuration object
        """
        self.config = config
        self.aero_model = SymbolicGdModel(config.aero_config)
        self._build_acados_model()
        self._setup_acados_sim()

    def _quat_multiply(self, q1, q2):
        """Quaternion multiplication: q1 * q2."""
        w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
        w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]

        return cs.vertcat(
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        )

    def _quat_to_rotmat(self, q):
        """Convert quaternion to rotation matrix."""
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]
        return cs.vertcat(
            cs.horzcat(1-2*(qy**2+qz**2), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy)),
            cs.horzcat(2*(qx*qy+qw*qz), 1-2*(qx**2+qz**2), 2*(qy*qz-qw*qx)),
            cs.horzcat(2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(qx**2+qy**2))
        )

    def _quat_to_rotmat_numpy(self, q):
        """Convert quaternion to rotation matrix (numpy version)."""
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]
        R = np.array([
            [1-2*(qy**2+qz**2), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy)],
            [2*(qx*qy+qw*qz), 1-2*(qx**2+qz**2), 2*(qy*qz-qw*qx)],
            [2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(qx**2+qy**2)]
        ])
        return R

    def _skew_symmetric(self, v):
        """Create skew-symmetric matrix from 3D vector."""
        return cs.vertcat(
            cs.horzcat(0, -v[2], v[1]),
            cs.horzcat(v[2], 0, -v[0]),
            cs.horzcat(-v[1], v[0], 0)
        )

    def _build_acados_model(self):
        """Build the ACADOS model with GD aerodynamics."""
        # Model name
        model_name = "gd_tailsitter_sim"

        # State variables
        p = cs.SX.sym('p', 3)  # Position [x, y, z]
        v = cs.SX.sym('v', 3)  # Velocity [vx, vy, vz]
        r = cs.SX.sym('r', 3)  # Angular rates [p, q, r]
        q = cs.SX.sym('q', 4)  # Quaternion [w, x, y, z]
        u = cs.SX.sym('u', 4)  # Motor thrusts [u1, u2, u3, u4]

        # Control input (motor rates)
        du = cs.SX.sym('du', 4)  # Motor thrust rates

        # State derivatives
        p_dot = cs.SX.sym('p_dot', 3)
        v_dot = cs.SX.sym('v_dot', 3)
        r_dot = cs.SX.sym('r_dot', 3)
        q_dot = cs.SX.sym('q_dot', 4)
        u_dot = cs.SX.sym('u_dot', 4)

        # Combine states and controls
        x = cs.vertcat(p, v, r, q, u)
        xdot = cs.vertcat(p_dot, v_dot, r_dot, q_dot, u_dot)

        # Constants
        mass = self.config.mass
        J = cs.diag(self.config.J)
        c_tau = self.config.c_tau
        dx, dy = self.config.dx, self.config.dy
        g = cs.vertcat(0.0, 0.0, -9.81)  # Gravity in NED frame

        # Transform quaternion from NED to world frame
        q_ned_to_world_sym = cs.SX(self.config.q_ned_to_world)
        q_world = self._quat_multiply(q_ned_to_world_sym, q)

        # Motor forces in body frame
        U = u[0] + u[1] + u[2] + u[3]
        thrust_body = cs.vertcat(0.0, 0.0, U)

        # Motor torques in body frame
        tau_body = cs.vertcat(
            dy * (-u[0] - u[1] + u[2] + u[3]),
            dx * (-u[0] + u[1] + u[2] - u[3]),
            c_tau * (-u[0] + u[1] - u[2] + u[3])
        )

        # Get velocity in body frame for aerodynamics
        R_world_to_body = self._quat_to_rotmat(q_world).T
        vel_body = cs.mtimes(R_world_to_body, v)

        # Transform to FRD frame for GD model evaluation
        q_flu_to_frd_sym = cs.SX(self.config.q_flu_to_frd)
        R_flu_to_frd = self._quat_to_rotmat(q_flu_to_frd_sym)
        vel_frd = cs.mtimes(R_flu_to_frd, vel_body)

        # Get aerodynamic forces and moments from GD model
        # The GD model expects body-frame velocity in FRD
        aero_result = self.aero_model.func(vel_frd)
        aero_forces_moments = aero_result[0]  # First element is forces_moments vector
        aero_forces_frd = aero_forces_moments[:3]  # First 3 elements are forces
        aero_moments_frd = aero_forces_moments[3:6]  # Next 3 elements are moments

        # Transform aerodynamic forces back to FLU frame, then to body frame
        aero_forces_flu = cs.mtimes(R_flu_to_frd.T, aero_forces_frd)

        # Use GD model forces directly
        aero_forces_body = aero_forces_flu

        # Total forces and moments in body frame
        total_forces_body = thrust_body + aero_forces_body
        total_moments_body = tau_body + aero_moments_frd  # Moments are frame-independent

        # Transform forces to world frame for dynamics
        R_body_to_world = self._quat_to_rotmat(q_world)
        forces_world = cs.mtimes(R_body_to_world, total_forces_body)

        # Dynamics equations
        p_dot_expr = v
        v_dot_expr = forces_world / mass + g
        r_dot_expr = cs.inv(J) @ (total_moments_body - cs.cross(r, J @ r))
        q_dot_expr = 0.5 * self._quat_multiply(cs.vertcat(0, r), q)
        u_dot_expr = du

        # Combine all dynamics
        f_expl = cs.vertcat(p_dot_expr, v_dot_expr, r_dot_expr, q_dot_expr, u_dot_expr)
        f_impl = xdot - f_expl

        # Create ACADOS model
        self.acados_model = AcadosModel()
        self.acados_model.f_impl_expr = f_impl
        self.acados_model.f_expl_expr = f_expl
        self.acados_model.x = x
        self.acados_model.xdot = xdot
        self.acados_model.u = du
        self.acados_model.p = cs.SX.sym('p', 0)  # No parameters needed
        self.acados_model.name = model_name

        # Store symbolic expressions for later use
        self._symbolic_vars = {
            'x': x,
            'v': v,
            'q': q,
            'u': u,
            'vel_body': vel_body,
            'vel_frd': vel_frd,
            'aero_forces_body': aero_forces_body,
            'total_forces_body': total_forces_body,
            'forces_world': forces_world
        }

    def _setup_acados_sim(self):
        """Setup ACADOS simulation solver."""
        # Create ACADOS simulation
        self.acados_sim = AcadosSim()
        self.acados_sim.model = self.acados_model

        # Configure solver options
        self.acados_sim.solver_options.T = self.config.dt
        self.acados_sim.solver_options.integrator_type = self.config.integrator_type
        self.acados_sim.solver_options.num_stages = self.config.num_stages
        self.acados_sim.solver_options.num_steps = self.config.num_steps
        self.acados_sim.solver_options.newton_iter = self.config.newton_iter
        self.acados_sim.solver_options.collocation_type = "GAUSS_RADAU_IIA"
        self.acados_sim.solver_options.sens_forw = True
        self.acados_sim.solver_options.sens_adj = False

        # Set parameter values (none needed for pure GD model)
        self.acados_sim.parameter_values = np.array([])  # No parameters

        # Create solver
        self.acados_solver = AcadosSimSolver(self.acados_sim)

    def simulate(self,
                 initial_state: np.ndarray,
                 control_trajectory: Optional[np.ndarray] = None) -> dict:
        """
        Run simulation with given initial state and control trajectory.

        Args:
            initial_state: Initial state vector [p, v, r, q, u]
            control_trajectory: Control input trajectory (N, 4)

        Returns:
            Dictionary containing simulation results
        """

        # Simulation setup
        N = int(self.config.sim_time / self.config.dt)
        nx = initial_state.shape[0]
        nu = 4  # Control input dimension

        # Storage for results
        states = np.zeros((N + 1, nx))
        controls = np.zeros((N, nu))
        times = np.linspace(0, self.config.sim_time, N + 1)

        # Set initial condition
        states[0, :] = initial_state

        # Set initial control (defaults to zero if not provided)
        if control_trajectory is None:
            control_trajectory = np.zeros((N, nu))
        else:
            control_trajectory = control_trajectory[:N, :]

        # Run simulation
        for i in range(N):
            # Set current state
            self.acados_solver.set("x", states[i, :])

            # Set control input
            self.acados_solver.set("u", control_trajectory[i, :])

            # Initialize implicit solver if needed
            if self.config.integrator_type == 'IRK':
                xdot_guess = np.zeros(nx)
                self.acados_solver.set("xdot", xdot_guess)

            # Solve integration step
            status = self.acados_solver.solve()

            if status != 0:
                raise Exception(f"ACADOS solver failed at step {i} with status {status}")

            # Store results
            states[i + 1, :] = self.acados_solver.get("x")
            controls[i, :] = control_trajectory[i, :]

        # Extract meaningful variables for post-processing
        results = {
            'states': states,
            'controls': controls,
            'times': times,
            'position': states[:, 0:3],
            'velocity': states[:, 3:6],
            'angular_rates': states[:, 6:9],
            'quaternion': states[:, 9:13],
            'motor_thrusts': states[:, 13:17]
        }

        return results

    def get_aerodynamic_forces(self, state: np.ndarray) -> dict:
        """
        Compute aerodynamic forces for a given state.

        Args:
            state: State vector [p, v, r, q, u]

        Returns:
            Dictionary with aerodynamic forces and moments
        """

        # Extract state variables
        v = state[3:6]
        q = state[9:13]

        # Transformations
        q_world = self._quat_multiply_numpy(self.config.q_ned_to_world, q)
        R_world_to_body = self._quat_to_rotmat_numpy(q_world).T
        vel_body = R_world_to_body @ v
        R_flu_to_frd = self._quat_to_rotmat_numpy(self.config.q_flu_to_frd)
        vel_frd = R_flu_to_frd @ vel_body

        # Get aerodynamic forces from GD model
        forces, moments, alpha, beta = self.aero_model.evaluate(vel_frd)

        # Transform forces back to body frame
        forces_body = R_flu_to_frd.T @ forces

        return {
            'forces_body': forces_body,
            'moments_body': moments,
            'alpha': alpha,
            'beta': beta,
            'vel_body': vel_body,
            'vel_frd': vel_frd
        }

    def _quat_multiply_numpy(self, q1, q2):
        """Quaternion multiplication (numpy version)."""
        w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
        w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]

        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])


def create_gd_simulator(config: GdSimConfig = DEFAULT_SIM_CONFIG) -> GdModelSimulator:
    """
    Factory function to create a GD model simulator.

    Args:
        config: Simulation configuration object

    Returns:
        GdModelSimulator instance
    """
    return GdModelSimulator(config)


def test_gd_simulation():
    """Test the GD model simulator with a simple flight scenario."""
    print("Testing GD Model Simulation...")

    # Create simulator with shorter time for debugging
    config = GdSimConfig(dt=0.01, sim_time=1.0)  # Shorter simulation
    sim = create_gd_simulator(config)

    # Initial state: hover at 10m altitude with some forward velocity
    p0 = np.array([0.0, 0.0, -10.0])  # Position (NED frame)
    v0 = np.array([5.0, 0.0, 0.0])    # Forward velocity
    r0 = np.array([0.0, 0.0, 0.0])    # Angular rates
    q0 = np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion (level)
    u0 = np.array([2.4525, 2.4525, 2.4525, 2.4525])  # Motor thrusts (hover)

    # Combine initial state (no cz_state needed)
    initial_state = np.concatenate([p0, v0, r0, q0, u0])

    # Control trajectory: steady hover
    N = int(sim.config.sim_time / sim.config.dt)
    control_trajectory = np.tile(u0, (N, 1))

    try:
        # Run simulation
        print(f"Running simulation with {N} steps...")
        results = sim.simulate(initial_state, control_trajectory)

        # Check for NaN values
        if np.any(np.isnan(results['states'])):
            print("❌ Simulation contains NaN values!")
            # Find first NaN occurrence
            nan_indices = np.where(np.isnan(results['states']))
            if len(nan_indices[0]) > 0:
                first_nan_step = nan_indices[0][0]
                print(f"   First NaN at step {first_nan_step}")
                print(f"   State at step {first_nan_step-1}: {results['states'][first_nan_step-1]}")
            return None

        print(f"✅ Simulation completed successfully!")
        print(f"   Time steps: {len(results['times'])}")
        print(f"   Final position: {results['position'][-1]}")
        print(f"   Final velocity: {results['velocity'][-1]}")
        print(f"   Max altitude: {np.max(-results['position'][:, 2]):.2f} m")
        print(f"   Min altitude: {np.min(-results['position'][:, 2]):.2f} m")

        # Get aerodynamic forces at final state
        aero_data = sim.get_aerodynamic_forces(results['states'][-1])
        print(f"   Final aero forces: {aero_data['forces_body']}")
        print(f"   Final alpha: {np.rad2deg(aero_data['alpha']):.2f} deg")
        print(f"   Final beta: {np.rad2deg(aero_data['beta']):.2f} deg")

        return results

    except Exception as e:
        print(f"❌ Simulation failed: {e}")
        raise


if __name__ == "__main__":
    test_gd_simulation()