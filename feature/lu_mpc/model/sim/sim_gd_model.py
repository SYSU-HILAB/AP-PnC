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
from ...utils.math_utils import quat_multiply, quat_apply, quat_apply_inverse


@dataclasses.dataclass
class GdSimConfig:
    """Configuration for the GD model simulation."""

    # Simulation parameters
    sim_dt: float = 0.02  # Time step in seconds

    # ACADOS solver options
    integrator_type: str = "ERK"  # 'IRK', 'ERK', 'GNSF'
    num_stages: int = 4  # Number of stages for implicit integrator
    num_steps: int = 20  # Number of steps per integration interval
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
    dx: float = 0.32  # Arm length x
    dy: float = 0.32  # Arm length y
    motor_tau: float = 0.01
    # FLU to FRD transformation
    trans_wing_frd_quad_flu: np.ndarray = np.array(
        [0.0, 0.7071, 0.0, 0.7071]
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
    """

    def __init__(self, init_state: np.ndarray, config: GdSimConfig):
        """
        Initialize the GD model simulator.

        Args:
            init_state: Initial state (17D array) [p(3), v(3), r(3), q(4), motor(4)]
            config: Simulation configuration object
        """
        # Validate state shape
        expected_shape = (17,)
        if init_state.shape != expected_shape:
            raise ValueError(
                f"init_state must have shape {expected_shape}, got {init_state.shape}"
            )

        self._cur_state = init_state.astype(np.float64).copy()
        self._config = config
        self._aero_model = SymbolicGdModel(config.aero_config)
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
        motor_thrusts = cs.SX.sym("motor_thrusts", 4)  # Motor thrusts [u1, u2, u3, u4]


        motor_thrusts_cmds = cs.SX.sym("motor_thrusts_cmds", 4)
        # Control input (motor rates)
        u = motor_thrusts_cmds

        # State derivatives
        p_dot = cs.SX.sym("p_dot", 3)
        v_dot = cs.SX.sym("v_dot", 3)
        r_dot = cs.SX.sym("r_dot", 3)
        q_dot = cs.SX.sym("q_dot", 4)
        motor_thrusts_dot = cs.SX.sym("motor_thrusts_dot", 4)

        # Combine states and controls
        x = cs.vertcat(p, v, r, q, motor_thrusts)  # quad frames
        xdot = cs.vertcat(p_dot, v_dot, r_dot, q_dot, motor_thrusts_dot)

        # Constants
        mass = self._config.mass
        J = cs.diag(self._config.J)
        c_tau = self._config.c_tau
        dx, dy = self._config.dx, self._config.dy
        g = cs.vertcat(0.0, 0.0, -9.81)  # Gravity in ENU frame

        quat_quad_flu = q

        trans_wing_frd_quad_flu = cs.SX(self._config.trans_wing_frd_quad_flu)
        quat_wing_frd = quat_multiply(quat_quad_flu, trans_wing_frd_quad_flu)

        # Motor forces in body frame
        U = motor_thrusts[0] + motor_thrusts[1] + motor_thrusts[2] + motor_thrusts[3]
        thrust_quad_flu = cs.vertcat(0.0, 0.0, U)

        # Motor torques in body frame
        tau_quad_flu = cs.vertcat(
            dy * (-motor_thrusts[0] - motor_thrusts[1] + motor_thrusts[2] + motor_thrusts[3]),
            dx * (-motor_thrusts[0] + motor_thrusts[1] + motor_thrusts[2] - motor_thrusts[3]),
            c_tau * (-motor_thrusts[0] + motor_thrusts[1] - motor_thrusts[2] + motor_thrusts[3]),
        )

        # Get velocity in body frame for aerodynamics
        vel_b_wing_frd = quat_apply_inverse(quat_wing_frd, v)

        # Get aerodynamic forces and moments from GD model
        # The GD model expects body-frame velocity in FRD
        aero_result = self._aero_model.func(vel_b_wing_frd)
        aero_forces_moments = aero_result[0]  # First element is forces_moments vector
        aero_forces_wing_frd = aero_forces_moments[:3]  # First 3 elements are forces
        aero_moments_wing_frd = aero_forces_moments[3:6]  # Next 3 elements are moments

        # Transform aerodynamic forces back to FLU frame, then to body frame
        # Use inverse transformation to go from wing FRD to quad FLU
        aero_forces_quad_flu = quat_apply(
            trans_wing_frd_quad_flu, aero_forces_wing_frd
        )
        aero_moments_quad_flu = quat_apply(
            trans_wing_frd_quad_flu, aero_moments_wing_frd
        )
        # Use GD model forces directly
        aero_forces_quad_flu = aero_forces_quad_flu

        # Total forces and moments in body frame
        tot_force_quad_flu = thrust_quad_flu + aero_forces_quad_flu
        # REC: currently not adding aerodynamic moments
        total_moments_quad_flu = tau_quad_flu
        # total_moments_quad_flu = (
        #     tau_quad_flu + aero_moments_quad_flu
        # )  # Moments are frame-independent

        # Transform forces to world frame for dynamics
        force_w = quat_apply(quat_quad_flu, tot_force_quad_flu)

        
        # Dynamics equations
        # Combine all dynamics
        f_expl = cs.vertcat(
            v,
            force_w / mass + g,
            cs.inv(J) @ (total_moments_quad_flu - cs.cross(r, J @ r)),
            0.5 * quat_multiply(quat_quad_flu, cs.vertcat(0, r)),
            (motor_thrusts_cmds - motor_thrusts) / self._config.motor_tau,
        )
        f_impl = xdot - f_expl

        # Create ACADOS model
        self._acados_model = AcadosModel()
        self._acados_model.f_impl_expr = f_impl
        self._acados_model.f_expl_expr = f_expl
        self._acados_model.x = x
        self._acados_model.xdot = xdot
        self._acados_model.u = motor_thrusts_cmds
        self._acados_model.name = model_name


    def _setup_acados_sim(self):
        """Setup ACADOS simulation solver."""
        # Create ACADOS simulation
        self._acados_sim: AcadosSim = AcadosSim()
        self._acados_sim.model = self._acados_model

        # Configure solver options
        self._acados_sim.solver_options.T = self._config.sim_dt
        self._acados_sim.solver_options.integrator_type = self._config.integrator_type
        self._acados_sim.solver_options.num_stages = self._config.num_stages
        self._acados_sim.solver_options.num_steps = self._config.num_steps
        self._acados_sim.solver_options.newton_iter = self._config.newton_iter
        self._acados_sim.solver_options.sens_forw = True
        self._acados_sim.solver_options.sens_adj = True

        # Set parameter values (none needed for pure GD model)
        self._acados_sim.parameter_values = np.array([])  # No parameters

        # Create solver
        self._acados_solver = AcadosSimSolver(self._acados_sim)


    @classmethod
    def create_gd_simulator(
        cls,
        init_state: np.ndarray,
        config: GdSimConfig,
    ) -> "GdModelSimulator":
        """
        Factory classmethod to create a GD model simulator.

        Args:
            init_state: Initial state (17D array) [p(3), v(3), r(3), q(4), motor(4)]
            config: Simulation configuration object

        Returns:
            GdModelSimulator instance
        """
        return cls(init_state, config)

    def simulate(self, motors_cmd: np.ndarray) -> np.ndarray:
        """
        Step simulation forward by one timestep (infinite simulate).
        Updates internal state and returns the new state.

        Args:
            motors_cmd: Motor commands (4D array) [m1, m2, m3, m4]

        Returns:
            next_state: State after simulation step (17D array)
        """
        # Validate input shape
        if motors_cmd.shape != (4,):
            raise ValueError(
                f"motors_cmd must have shape (4,), got {motors_cmd.shape}"
            )

        # Set current state and control input to solver
        self._acados_solver.set("x", self._cur_state)
        self._acados_solver.set("u", motors_cmd)

        # Solve the dynamics
        status = self._acados_solver.solve()

        if status != 0:
            raise RuntimeError(f"ACADOS solver failed with status {status}")

        # Update internal state (copy to avoid reference to solver buffer)
        self._cur_state = self._acados_solver.get("x").copy()

        return self._cur_state.copy()

    def get_state(self) -> np.ndarray:
        """
        Get current state.

        Returns:
            Current state (17D array) [p(3), v(3), r(3), q(4), motor(4)]
        """
        return self._cur_state.copy()
