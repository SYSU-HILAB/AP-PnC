"""Controller module for lu_mpc - wraps models into functional controllers."""
from .trajectory_loader import load_trajectory_csv, TrajectoryReference

__all__ = [
    "load_trajectory_csv",
    "TrajectoryReference",
    "ProposedNmpcController",
    "ErrStateLmpcController",
]

# Lazy import to avoid circular dependency with core.ocp_setup
# (ocp_setup imports controller.core.config, which triggers this __init__)
def __getattr__(name: str):
    if name == "ProposedNmpcController":
        from .ctrl_proposed_nmpc import ProposedNmpcController
        return ProposedNmpcController
    if name == "ErrStateLmpcController":
        from .ctrl_err_state_lmpc import ErrStateLmpcController
        return ErrStateLmpcController
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
