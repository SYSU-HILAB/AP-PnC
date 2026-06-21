"""Controller module for lu_mpc - wraps models into functional controllers."""
from .trajectory_loader import TrajectoryReference, load_trajectory_csv

__all__ = [
    "load_trajectory_csv",
    "TrajectoryReference",
    "ErrStateLmpcAngVelActuatedController",
    "ProposedNmpcAngVelActuatedController",
]

# Lazy import to avoid circular dependency with core.ocp_setup
# (ocp_setup imports controller.core.config, which triggers this __init__)
def __getattr__(name: str):
    if name == "ErrStateLmpcAngVelActuatedController":
        from .ctrl_err_state_lmpc_ang_vel_actuated import ErrStateLmpcAngVelActuatedController
        return ErrStateLmpcAngVelActuatedController
    if name == "ProposedNmpcAngVelActuatedController":
        from .ctrl_proposed_nmpc_ang_vel_actuated import ProposedNmpcAngVelActuatedController
        return ProposedNmpcAngVelActuatedController
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
