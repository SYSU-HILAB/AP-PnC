"""
Configuration for CtrlProperAccelZAngVel3D.

This module contains configuration class for CtrlProperAccelZAngVel3D.
Sub-controller configurations (PD, PID, QP Mixer) are imported from
controller.config.sub_controllers.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING
import numpy as np

from .sub_controllers import (
    PDControllerConfig,
    PIDAntiWindupConfig,
    QpMixerConfig,
)

if TYPE_CHECKING:
    from ..trajectory_loader import TrajectoryReference


# =============================================================================
# Proper Acceleration Z and Angular Velocity 3D Controller Configuration (combines all above)
# =============================================================================

@dataclass
class CtrlProperAccelZAngVel3DConfig:
    """
    Configuration for CtrlProperAccelZAngVel3D.

    Combines MPC configuration with PD, PID, and QP mixer configurations.
    """

    # MPC configuration (from ocp_proposed_mpc_cfg.py)
    mpc_config: object | None = None  # Will be set to MpcConfig instance

    # Trajectory reference
    trajectory: TrajectoryReference | None = None

    # Sub-controller configurations
    pd_config: PDControllerConfig = field(default_factory=PDControllerConfig)
    pid_config: PIDAntiWindupConfig = field(default_factory=PIDAntiWindupConfig)
    qp_mixer_config: QpMixerConfig = field(default_factory=QpMixerConfig)

    # ACADOS solver cache path
    json_path: Path = field(
        default_factory=lambda: Path(__file__).parent.parent.parent / ".cache" / "acados_ocp_ctrl_proper_accel_z_ang_vel_3d.json"
    )

    # Control mode
    is_hover_task: bool = False  # If True, hover at initial reference

    # Gravity vector (for proper acceleration computation)
    gravity: np.ndarray = field(default_factory=lambda: np.array([0, 0, -9.81]))

    @classmethod
    def create_default(cls, trajectory: TrajectoryReference) -> CtrlProperAccelZAngVel3DConfig:
        """Create default angular velocity controller configuration."""
        from .ocp_proposed_mpc_cfg import MpcConfig

        return cls(
            mpc_config=MpcConfig.create_high_performance(),
            trajectory=trajectory,
        )