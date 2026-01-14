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
from typing import Optional
from acados_template import AcadosModel, AcadosSim, AcadosSimSolver
from .gd_model_casadi import SymbolicGdModel, GdModelConfig, DEFAULT_CONFIG
from feature.utils.math_utils import quat_multiply, quat_apply, quat_apply_inverse


@dataclasses.dataclass
class GdSimConfig:
    """Configuration for the GD model simulation."""

    # Simulation parameters
    dt: float = 0.01  # Time step in seconds
    sim_time: float = 5.0  # Total simulation time in seconds

    # ACADOS solver options
    integrator_type: str = "ERK"  # 'IRK', 'ERK', 'GNSF'
    num_stages: int = 4  # Number of stages for implicit integrator
    num_steps: int = 1  # Number of steps per integration interval
    newton_iter: int = 3  # Newton iterations for implicit solver
    #   <turningDirection>cw</turningDirection>
    #   <timeConstantUp>0.0125</timeConstantUp>
    #   <timeConstantDown>0.025</timeConstantDown>
    #   <maxRotVelocity>2000.0</maxRotVelocity>
    #   <motorConstant>8.54858e-06</motorConstant>
    #   <momentConstant>0.032</momentConstant>
    # <joint name='rotor_2_joint' type='revolute'>
    #   <pose relative_to="base_link">-0.015 0.32 -0.32 0 1.5708 0</pose>

    # Aircraft physical parameters (from tailsitter_flu)
    mass: float = 2.0  # kg
    J: np.ndarray = np.array([0.04361, 0.04513, 0.08322])  # Inertia matrix
    c_tau: float = 0.032  # Torque coefficient
    dx: float = 0.08838834764831845  # Arm length x
    dy: float = 0.08838834764831845  # Arm length y

    # # Frame transformation quaternions
    # q_flu_to_frd: np.ndarray = np.array([0.0, 1.0, 0.0, 0.0])

    # FLU to FRD transformation
    trans_wing_frd_quad_flu: np.ndarray = np.array(
        [0.7071, 0.0, 0.7071, 0.0]
    )  # NED to world transformation

    # Aerodynamic model configuration
    aero_config: GdModelConfig = DEFAULT_CONFIG


# Default configuration instance
DEFAULT_SIM_CONFIG = GdSimConfig()


class GdModelSimulator:
    """High-fidelity tailsitter simulator using GD aerodynamics model.
       We will assume the data will be generated in quadrotor local world frame,
       where its (0,0,0) is start point in the world;
            the gravity vector is [0,0,-9.81] in the frame.
    ️   However, the gd aerodynamics is defined in wing body frame (FRD),
       this frame to typical quadrotor local FLU frame transformation is defined in config.
       i.e., q_wing_frd_to_quad_flu.
       It means, to get the wing body frame velocity, we need to:
        1) transform the velocity from world frame to quad local FLU frame (using current quaternion);
        2) transform the velocity from quad local FLU frame to wing body FRD
        3) transform the wing body FRD aerodynamics to quad local FLU frame for force summation.
    """

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

    def _build_acados_model(self):
        """Build the ACADOS model with GD aerodynamics."""
        # Model name
        model_name = "gd_tailsitter_sim"

        # State variables
        p = cs.SX.sym("p", 3)  # Position [x, y, z]
        v = cs.SX.sym("v", 3)  # Velocity [vx, vy, vz]
        r = cs.SX.sym("r", 3)  # Angular rates [p, q, r]
        q = cs.SX.sym("q", 4)  # Quaternion [w, x, y, z]
        u = cs.SX.sym("u", 4)  # Motor thrusts [u1, u2, u3, u4]

        # Control input (motor rates)
        du = cs.SX.sym("du", 4)  # Motor thrust rates

        # State derivatives
        p_dot = cs.SX.sym("p_dot", 3)
        v_dot = cs.SX.sym("v_dot", 3)
        r_dot = cs.SX.sym("r_dot", 3)
        q_dot = cs.SX.sym("q_dot", 4)
        u_dot = cs.SX.sym("u_dot", 4)

        # Combine states and controls
        x = cs.vertcat(p, v, r, q, u)  # quad frames
        xdot = cs.vertcat(p_dot, v_dot, r_dot, q_dot, u_dot)

        # Constants
        mass = self.config.mass
        J = cs.diag(self.config.J)
        c_tau = self.config.c_tau
        dx, dy = self.config.dx, self.config.dy
        g = cs.vertcat(0.0, 0.0, -9.81)  # Gravity in NED frame

        q_quad_flu = q

        trans_wing_frd_quad_flu = cs.SX(self.config.trans_wing_frd_quad_flu)
        q_wing_frd = quat_multiply(q_quad_flu, trans_wing_frd_quad_flu)

        # Motor forces in body frame
        U = u[0] + u[1] + u[2] + u[3]
        thrust_body = cs.vertcat(0.0, 0.0, U)

        # Motor torques in body frame
        tau_body = cs.vertcat(
            dy * (-u[0] - u[1] + u[2] + u[3]),
            dx * (-u[0] + u[1] + u[2] - u[3]),
            c_tau * (-u[0] + u[1] - u[2] + u[3]),
        )

        # Get velocity in body frame for aerodynamics
        vel_b_wing_frd = quat_apply(q_wing_frd, v)

        # Get aerodynamic forces and moments from GD model
        # The GD model expects body-frame velocity in FRD
        aero_result = self.aero_model.func(vel_b_wing_frd)
        aero_forces_moments = aero_result[0]  # First element is forces_moments vector
        aero_forces_wing_frd = aero_forces_moments[:3]  # First 3 elements are forces
        aero_moments_wing_frd = aero_forces_moments[3:6]  # Next 3 elements are moments

        # Transform aerodynamic forces back to FLU frame, then to body frame
        # Use inverse transformation to go from wing FRD to quad FLU
        aero_forces_quad_flu = quat_apply_inverse(
            trans_wing_frd_quad_flu, aero_forces_wing_frd
        )
        aero_moments_quad_flu = quat_apply_inverse(
            trans_wing_frd_quad_flu, aero_moments_wing_frd
        )
        # Use GD model forces directly
        aero_forces_body = aero_forces_quad_flu

        # Total forces and moments in body frame
        total_forces_body = thrust_body + aero_forces_body
        total_moments_body = (
            tau_body + aero_moments_quad_flu
        )  # Moments are frame-independent
        # Transform forces to world frame for dynamics
        forces_world = quat_apply(q_quad_flu, total_forces_body)

        # Dynamics equations
        p_dot_expr = v
        v_dot_expr = forces_world / mass + g
        r_dot_expr = cs.inv(J) @ (total_moments_body - cs.cross(r, J @ r))
        q_dot_expr = 0.5 * quat_multiply(cs.vertcat(0, r), q_quad_flu)
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
        self.acados_model.p = cs.SX.sym("p", 0)  # No parameters needed
        self.acados_model.name = model_name

        # Store symbolic expressions for later use
        self._symbolic_vars = {
            "x": x,
            "v": v,
            "q": q_quad_flu,
            "u": u,
            "vel_wing_frd": vel_b_wing_frd,
            "aero_forces_body": aero_forces_body,
            "total_forces_body": total_forces_body,
            "forces_world": forces_world,
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

    def simulate(
        self,
        initial_state: np.ndarray,
        control_trajectory: Optional[np.ndarray] = None,
        collect_aero_data: bool = False,
    ) -> dict:
        """
        Run simulation with given initial state and control trajectory.

        Args:
            initial_state: Initial state vector [p, v, r, q, u]
            control_trajectory: Control input trajectory (N, 4)
            collect_aero_data: Whether to collect aerodynamic data at each step

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
        aero_data_list = [] if collect_aero_data else None

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
            if self.config.integrator_type == "IRK":
                xdot_guess = np.zeros(nx)
                self.acados_solver.set("xdot", xdot_guess)

            # Solve integration step
            status = self.acados_solver.solve()

            if status != 0:
                raise Exception(
                    f"ACADOS solver failed at step {i} with status {status}"
                )

            # Store results
            states[i + 1, :] = self.acados_solver.get("x")
            controls[i, :] = control_trajectory[i, :]

            # Collect aerodynamic data if requested
            if collect_aero_data:
                aero_data = self.get_aerodynamic_forces(states[i + 1, :])
                aero_data_list.append(aero_data)

        # Extract meaningful variables for post-processing
        results = {
            "states": states,
            "controls": controls,
            "times": times,
            "position": states[:, 0:3],
            "velocity": states[:, 3:6],
            "angular_rates": states[:, 6:9],
            "quaternion": states[:, 9:13],
            "motor_thrusts": states[:, 13:17],
        }

        if collect_aero_data:
            results["aero_data"] = aero_data_list

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
        q_quad_flu = state[9:13]

        # Transformations: world -> quad FLU -> wing FRD
        # World to quad FLU (inverse of quad FLU to world)
        vel_quad_flu = quat_apply_inverse(q_quad_flu, v)

        # Quad FLU to wing FRD
        trans_wing_frd_quad_flu = self.config.trans_wing_frd_quad_flu
        vel_wing_frd = quat_apply(trans_wing_frd_quad_flu, vel_quad_flu)

        # Get aerodynamic forces from GD model
        aero_result = self.aero_model.func(cs.DM(vel_wing_frd))
        aero_forces_moments = aero_result[0]
        forces_wing_frd = np.array(aero_forces_moments[:3]).flatten()
        moments_wing_frd = np.array(aero_forces_moments[3:6]).flatten()

        # Transform forces back to quad FLU frame
        forces_quad_flu = quat_apply_inverse(trans_wing_frd_quad_flu, forces_wing_frd)

        # Calculate alpha and beta from velocity
        vx, vy, vz = vel_wing_frd
        v_total = np.sqrt(vx**2 + vy**2 + vz**2)
        if v_total > 1e-6:
            alpha = np.arctan2(vz, vx)
            beta = np.arcsin(np.clip(vy / v_total, -1, 1))
        else:
            alpha = 0.0
            beta = 0.0

        return {
            "forces_body": forces_quad_flu,
            "moments_body": moments_wing_frd,
            "alpha": alpha,
            "beta": beta,
            "vel_body": vel_quad_flu,
            "vel_frd": vel_wing_frd,
        }


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
    v0 = np.array([5.0, 0.0, 0.0])  # Forward velocity
    r0 = np.array([0.0, 0.0, 0.0])  # Angular rates
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
        if np.any(np.isnan(results["states"])):
            print("❌ Simulation contains NaN values!")
            # Find first NaN occurrence
            nan_indices = np.where(np.isnan(results["states"]))
            if len(nan_indices[0]) > 0:
                first_nan_step = nan_indices[0][0]
                print(f"   First NaN at step {first_nan_step}")
                print(
                    f"   State at step {first_nan_step - 1}: {results['states'][first_nan_step - 1]}"
                )
            return None

        print("✅ Simulation completed successfully!")
        print(f"   Time steps: {len(results['times'])}")
        print(f"   Final position: {results['position'][-1]}")
        print(f"   Final velocity: {results['velocity'][-1]}")
        print(f"   Max altitude: {np.max(-results['position'][:, 2]):.2f} m")
        print(f"   Min altitude: {np.min(-results['position'][:, 2]):.2f} m")

        # Get aerodynamic forces at final state
        aero_data = sim.get_aerodynamic_forces(results["states"][-1])
        print(f"   Final aero forces: {aero_data['forces_body']}")
        print(f"   Final alpha: {np.rad2deg(aero_data['alpha']):.2f} deg")
        print(f"   Final beta: {np.rad2deg(aero_data['beta']):.2f} deg")

        return results

    except Exception as e:
        print(f"❌ Simulation failed: {e}")
        raise


if __name__ == "__main__":
    test_gd_simulation()
