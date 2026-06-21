"""Sub-controller modules for the lu_mpc controller."""

__all__ = [
    "PDController",
    "PIDAntiWindupController",
    "QpMixer",
]


def __getattr__(name: str):
    if name == "PDController":
        from .pd_controller import PDController
        return PDController
    if name == "PIDAntiWindupController":
        from .pid_anti_windup_controller import PIDAntiWindupController
        return PIDAntiWindupController
    if name == "QpMixer":
        from .qp_mixer import QpMixer
        return QpMixer
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
