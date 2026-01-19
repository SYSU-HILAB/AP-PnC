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

import numpy as np

from ..sim.gd_model_casadi import SymbolicGdModel, GdModelConfig, DEFAULT_CONFIG
from ....utils.math_utils import quat_multiply, quat_apply, quat_apply_inverse


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
        forces, moments, alpha, beta = self.model.evaluate(vel_b_wing_frd)

        # Step 4: Transform forces from wing FRD to quad FLU frame
        f_a_quad_flu = quat_apply(self.TRANS_WING_FRD_TO_QUAD_FLU, forces)

        return f_a_quad_flu
