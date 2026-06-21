"""
Aerodynamic force evaluator for error state MPC.

Evaluates aerodynamic forces using the full GD model and provides
them as parameters to the error state MPC.

Frame Conventions:
- Quad FLU: Standard quadrotor body frame (Front-Left-Up)
- Wing FRD: Wing body frame for GD aerodynamics (Front-Right-Down)
- World frame: ENU (East-North-Up) with gravity [0, 0, -9.81]
"""

from __future__ import annotations

import casadi as cs
import numpy as np

from ....utils.math_utils import quat_apply, quat_apply_inverse, quat_multiply
from ..sim.gd_model_casadi import DEFAULT_CONFIG, GdModelConfig, SymbolicGdModel


class AeroForceEvaluator:
    """Evaluates aerodynamic forces using the full GD model."""

    # Transformation from wing FRD to quad FLU (from sim_gd_model.py)
    # This rotates the FRD (Front-Right-Down) wing frame to FLU (Front-Left-Up) quad frame
    TRANS_WING_FRD_TO_QUAD_FLU = np.array([0.0, 0.7071, 0.0, 0.7071])

    def __init__(self, config: GdModelConfig | None = None):
        """
        Initialize the evaluator.

        Args:
            config: GD model configuration (uses default if None)
        """
        self.model = SymbolicGdModel(config or DEFAULT_CONFIG)
        self._build_jacobian_function()
        self._build_body_forces_jacobian_alpha_function()

    def _build_jacobian_function(self):
        """
        Build CasADi function for computing Jacobian using automatic differentiation.

        Creates a symbolic function that computes forces in world frame
        from world velocity and orientation, then computes the Jacobian.
        """
        # Symbolic inputs
        self._v_world_sym = cs.SX.sym("v_world", 3)  # World velocity
        self._quat_quad_flu_sym = cs.SX.sym("quat_quad_flu", 4)  # Quad orientation

        # Convert transformation to CasADi
        trans_wing_to_quad = cs.SX(self.TRANS_WING_FRD_TO_QUAD_FLU)

        # Step 1: Compute wing orientation (FRD) from quad orientation (FLU)
        quat_wing_frd = self._quat_multiply_sym(
            self._quat_quad_flu_sym, trans_wing_to_quad
        )

        # Step 2: Transform world velocity to wing body frame (FRD)
        vel_b_wing_frd = self._quat_apply_inverse_sym(quat_wing_frd, self._v_world_sym)

        # Step 3: Get forces from the model's symbolic expressions
        # The model has symbolic forces that depend on velb
        # We need to substitute the model's velb with our vel_b_wing_frd
        forces_wing_frd = cs.substitute(
            self.model.forces,
            self.model.velb,
            vel_b_wing_frd
        )

        # Step 4: Transform forces from wing FRD to quad FLU frame
        f_a_quad_flu = self._quat_apply_sym(trans_wing_to_quad, forces_wing_frd)

        # Step 5: Transform forces from quad FLU to world frame
        f_a_world = self._quat_apply_sym(self._quat_quad_flu_sym, f_a_quad_flu)

        # Store symbolic forces (in world frame for Jacobian)
        self._forces_sym = f_a_world

        # Compute Jacobian using CasADi automatic differentiation
        # Jacobian is ∂fa_world/∂v_world
        jacobian_sym = cs.jacobian(self._forces_sym, self._v_world_sym)

        # Create compiled function
        self._jacobian_func = cs.Function(
            "forces_jacobian",
            [self._v_world_sym, self._quat_quad_flu_sym],
            [jacobian_sym],
        )

    def _build_body_forces_jacobian_alpha_function(self):
        """
        Build CasADi function for computing ∂B_fa/∂alpha Jacobian.

        Uses simplified velocity model: velb = [V*cos(alpha), 0, V*sin(alpha)].
        The Jacobian is a 3x1 vector (derivative of 3D force vector with respect to scalar alpha).
        """
        # Symbolic inputs
        self._V_sym = cs.SX.sym("V")  # Velocity magnitude
        self._alpha_sym = cs.SX.sym("alpha")  # Angle of attack

        # Construct body velocity: [V*cos(alpha), 0, V*sin(alpha)]
        velb_sym = cs.vertcat(
            self._V_sym * cs.cos(self._alpha_sym),
            0.0,
            self._V_sym * cs.sin(self._alpha_sym)
        )

        # Get body-frame forces from model (B_fa in WING_FRD frame)
        # Substitute model's velb with our constructed velb_sym
        B_fa_sym = cs.substitute(
            self.model.forces,
            self.model.velb,
            velb_sym
        )

        # Compute Jacobian: ∂B_fa/∂alpha (3x1 vector)
        jacobian_sym = cs.jacobian(B_fa_sym, self._alpha_sym)

        # Create compiled function
        self._body_forces_jacobian_alpha_func = cs.Function(
            "body_forces_jacobian_alpha",
            [self._V_sym, self._alpha_sym],
            [jacobian_sym],
        )

    def _quat_multiply_sym(
        self, q1: cs.SX, q2: cs.SX
    ) -> cs.SX:
        """CasADi symbolic quaternion multiplication."""
        w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
        w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]
        return cs.vertcat(
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        )

    def _quat_apply_sym(self, q: cs.SX, v: cs.SX) -> cs.SX:
        """Apply CasADi symbolic quaternion rotation to a vector."""
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]
        q_xyz = cs.vertcat(qx, qy, qz)
        t = 2.0 * cs.cross(q_xyz, v)
        return v + qw * t + cs.cross(q_xyz, t)

    def _quat_apply_inverse_sym(self, q: cs.SX, v: cs.SX) -> cs.SX:
        """Apply CasADi symbolic inverse quaternion rotation to a vector."""
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]
        q_xyz = cs.vertcat(qx, qy, qz)
        t = 2.0 * cs.cross(q_xyz, v)
        return v - qw * t + cs.cross(q_xyz, t)

    def evaluate_forces(
        self, v_world: np.ndarray, quat_quad_flu: np.ndarray
    ) -> np.ndarray:
        """
        Evaluate aerodynamic forces for given velocity and orientation.

        Args:
            v_world: World frame velocity (3,) [vx, vy, vz] in m/s
            quat_quad_flu: Quaternion [w, x, y, z] from quad FLU to world

        Returns:
            f_a_quad_flu: Aerodynamic forces in quad FLU body frame (3,) [Fx, Fy, Fz] in N
        """
        # Step 1: Get wing orientation (FRD) from quad orientation (FLU)
        quat_wing_frd = quat_multiply(quat_quad_flu, self.TRANS_WING_FRD_TO_QUAD_FLU)

        # Step 2: Transform world velocity to wing body frame (FRD)
        vel_b_wing_frd = quat_apply_inverse(quat_wing_frd, v_world)

        # Step 3: Evaluate GD model (returns forces in wing FRD frame)
        forces_b, moments, alpha, beta = self.model.evaluate(vel_b_wing_frd)

        # Step 4: Transform forces from wing FRD to quad FLU frame
        f_a_quad_flu = quat_apply(self.TRANS_WING_FRD_TO_QUAD_FLU, forces_b)

        return f_a_quad_flu

    def compute_forces_jacobian(
        self, v_world: np.ndarray, quat_quad_flu: np.ndarray
    ) -> np.ndarray:
        """
        Compute the Jacobian of aerodynamic forces with respect to world velocity.

        The Jacobian is ∂fa_world/∂v_world, where:
        - fa_world: Aerodynamic forces in world frame
        - v_world: World frame velocity

        Uses CasADi automatic differentiation for exact computation.

        Args:
            v_world: World frame velocity (3,) [vx, vy, vz] in m/s
            quat_quad_flu: Quaternion [w, x, y, z] from quad FLU to world

        Returns:
            jacobian: Jacobian matrix (3, 3) where jacobian[i, j] = ∂fa_world[i]/∂v_world[j]
        """
        # Evaluate using the compiled CasADi function
        jacobian_result = self._jacobian_func(v_world, quat_quad_flu)
        return jacobian_result.full()

    def compute_body_forces_jacobian_alpha(
        self, V: float, alpha: float
    ) -> np.ndarray:
        """
        Compute the Jacobian of body-frame aerodynamic forces with respect to angle of attack.

        The Jacobian is ∂B_fa/∂alpha, where:
        - B_fa: Aerodynamic forces in body frame (3D vector in WING_FRD)
        - alpha: Angle of attack (scalar in radians)

        Uses the simplified velocity model: velb = [V*cos(alpha), 0, V*sin(alpha)]

        Args:
            V: Velocity magnitude (m/s)
            alpha: Angle of attack (radians)

        Returns:
            Jacobian vector (3,) where jacobian[i] = ∂B_fa[i]/∂alpha
        """
        # Evaluate using the compiled CasADi function
        jacobian_result = self._body_forces_jacobian_alpha_func(V, alpha)
        return jacobian_result.full().flatten()
