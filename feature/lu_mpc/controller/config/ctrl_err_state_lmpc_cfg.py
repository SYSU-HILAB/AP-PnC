"""
Configuration for angular-velocity-actuated error state linear MPC.

This module contains configuration class for ErrStateLmpcAngVelActuatedCfg.
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
# Angular-Velocity-Actuated Error State Linear MPC Controller Configuration (combines all above)
# =============================================================================

@dataclass
class ErrStateLmpcAngVelActuatedCfg:
    """
    Unified configuration for angular-velocity-actuated error state linear MPC.

    Combines MPC configuration with PD, PID, and QP mixer configurations.
    """

    # MPC configuration (from ocp_err_state_cfg.py)
    mpc_config: object | None = None  # Will be set to ErrorStateLinearMpcConfig instance

    # Trajectory reference
    trajectory: TrajectoryReference | None = None

    # Sub-controller configurations
    pd_config: PDControllerConfig = field(default_factory=PDControllerConfig)
    pid_config: PIDAntiWindupConfig = field(default_factory=PIDAntiWindupConfig)
    qp_mixer_config: QpMixerConfig = field(default_factory=QpMixerConfig)

    # ACADOS solver cache path
    json_path: Path = field(
        default_factory=lambda: Path(__file__).parent.parent.parent / ".cache" / "acados_ocp_err_state_lmpc.json"
    )

    # Control mode
    is_hover_task: bool = False  # If True, hover at initial reference

    @classmethod
    def create_default(cls, trajectory: TrajectoryReference) -> ErrStateLmpcAngVelActuatedCfg:
        """Create default error state MPC controller configuration."""
        from .ocp_err_state_cfg import ErrorStateLinearMpcConfig

        return cls(
            mpc_config=ErrorStateLinearMpcConfig.create_default(),
            trajectory=trajectory,
        )

    @classmethod
    def create_high_gain(cls, trajectory: TrajectoryReference) -> ErrStateLmpcAngVelActuatedCfg:
        """Create high-gain error state MPC controller configuration."""
        from .ocp_err_state_cfg import ErrorStateLinearMpcConfig

        return cls(
            mpc_config=ErrorStateLinearMpcConfig.create_high_gain(),
            trajectory=trajectory,
        )
