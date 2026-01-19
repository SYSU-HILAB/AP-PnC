"""Controller module for lu_mpc - wraps models into functional controllers."""
from .trajectory_loader import load_trajectory_csv, TrajectoryReference

__all__ = [
    "load_trajectory_csv",
    "TrajectoryReference",
    "ErrStateLmpcController",
    "CtrlProperAccelZAngVel3D",
    "create_ctrl_proper_accel_z_ang_vel_3d",
]

# Lazy import to avoid circular dependency with core.ocp_setup
# (ocp_setup imports controller.core.config, which triggers this __init__)
def __getattr__(name: str):
    if name == "ErrStateLmpcController":
        from .ctrl_err_state_lmpc import ErrStateLmpcController
        return ErrStateLmpcController
    if name == "CtrlProperAccelZAngVel3D":
        from .ctrl_proper_accel_z_ang_vel_3d import CtrlProperAccelZAngVel3D
        return CtrlProperAccelZAngVel3D
    if name == "create_ctrl_proper_accel_z_ang_vel_3d":
        from .ctrl_proper_accel_z_ang_vel_3d import create_ctrl_proper_accel_z_ang_vel_3d
        return create_ctrl_proper_accel_z_ang_vel_3d
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
