"""
Configuration for CtrlProperAccelZAngVel3D.

This module contains configuration classes for:
- PD Controller (proper acceleration tracking)
- PID Controller with anti-windup (angular velocity tracking)
- QP Mixer (control allocation)
- CtrlProperAccelZAngVel3D (combines all above)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING
import numpy as np

if TYPE_CHECKING:
    from ..trajectory_loader import TrajectoryReference


# =============================================================================
# PD Controller Configuration (for proper acceleration)
# =============================================================================

@dataclass
class PDControllerConfig:
    """Configuration for PD controller tracking proper acceleration."""

    Kp: float = 1.0  # Proportional gain
    Kd: float = 0.1  # Derivative gain

    # Output limits
    output_min: float = 0.0  # Minimum total thrust (N)
    output_max: float = 40.0  # Maximum total thrust (N)

    @classmethod
    def create_default(cls) -> PDControllerConfig:
        """Create default PD configuration."""
        return cls()

    @classmethod
    def create_aggressive(cls) -> PDControllerConfig:
        """Create configuration for aggressive tracking."""
        return cls(
            Kp=2.0,
            Kd=0.2,
        )


# =============================================================================
# PID Controller with Anti-Windup Configuration (for angular velocity)
# =============================================================================

@dataclass
class PIDAntiWindupConfig:
    """Configuration for PID controller with anti-windup for angular velocity."""

    Kp: np.ndarray = field(default_factory=lambda: np.array([0.5, 0.5, 0.5]))  # Proportional gains
    Ki: np.ndarray = field(default_factory=lambda: np.array([0.1, 0.1, 0.1]))  # Integral gains
    Kd: np.ndarray = field(default_factory=lambda: np.array([0.05, 0.05, 0.05]))  # Derivative gains

    # Anti-windup settings
    integral_max: float = 1.0  # Maximum integral error (anti-windup)
    integral_min: float = -1.0  # Minimum integral error (anti-windup)

    # Output limits (torque limits)
    output_min: np.ndarray = field(default_factory=lambda: np.array([-1.0, -1.0, -0.5]))
    output_max: np.ndarray = field(default_factory=lambda: np.array([1.0, 1.0, 0.5]))

    # Timing
    dt: float = 0.02  # Controller time step (s)

    @classmethod
    def create_default(cls) -> PIDAntiWindupConfig:
        """Create default PID configuration."""
        return cls()

    @classmethod
    def create_high_gain(cls) -> PIDAntiWindupConfig:
        """Create high-gain configuration for faster tracking."""
        return cls(
            Kp=np.array([1.0, 1.0, 1.0]),
            Ki=np.array([0.2, 0.2, 0.2]),
            Kd=np.array([0.1, 0.1, 0.1]),
        )


# =============================================================================
# QP Mixer Configuration (from mixer_config.py)
# =============================================================================

@dataclass
class QpMixerConfig:
    """Configuration for QP control mixer."""

    # Physical parameters (must match simulator)
    dx: float = 0.32  # Arm length x (m)
    dy: float = 0.32  # Arm length y (m)
    c_tau: float = 0.032  # Torque coefficient

    # Motor constraints (from PX4 motor specs)
    u_min: float = 0.0  # Minimum motor thrust (N)
    u_max: float = 10.0  # Maximum motor thrust (N)
    du_min: float = -10.0  # Minimum motor rate (N/s)
    du_max: float = 10.0  # Maximum motor rate (N/s)

    # Control timing (should match control frequency)
    dt: float = 0.02  # Time step for motor dynamics (s)

    # QP weights (tunable for performance)
    W_tracking: np.ndarray = field(default_factory=lambda: np.eye(4))
    R_prev_cmd: float = 0.1  # Weight on command smoothness

    @classmethod
    def create_default(cls) -> QpMixerConfig:
        """Create default mixer configuration."""
        return cls()

    @classmethod
    def create_aggressive(cls) -> QpMixerConfig:
        """Create configuration for aggressive maneuvering."""
        return cls(
            W_tracking=np.diag([10.0, 5.0, 5.0, 1.0]),  # Prioritize thrust
            R_prev_cmd=0.01,  # Allow faster changes
        )

    @classmethod
    def create_smooth(cls) -> QpMixerConfig:
        """Create configuration for smooth, efficient flight."""
        return cls(
            W_tracking=np.diag([1.0, 1.0, 1.0, 1.0]),
            R_prev_cmd=1.0,  # Strong smoothness
        )


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

    @classmethod
    def create_aggressive(cls, trajectory: TrajectoryReference) -> CtrlProperAccelZAngVel3DConfig:
        """Create aggressive tracking configuration."""
        from .ocp_proposed_mpc_cfg import MpcConfig

        return cls(
            mpc_config=MpcConfig.create_high_performance(),
            trajectory=trajectory,
            pd_config=PDControllerConfig.create_aggressive(),
            pid_config=PIDAntiWindupConfig.create_high_gain(),
            qp_mixer_config=QpMixerConfig.create_aggressive(),
        )


# Default configuration instances (requires trajectory at runtime)
DEFAULT_PD_CONFIG = PDControllerConfig()
DEFAULT_PID_CONFIG = PIDAntiWindupConfig()
DEFAULT_QP_MIXER_CONFIG = QpMixerConfig()
