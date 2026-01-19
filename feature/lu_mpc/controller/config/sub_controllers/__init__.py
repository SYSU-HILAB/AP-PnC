"""Sub-controller configuration modules for the lu_mpc controller."""

__all__ = [
    "PDControllerConfig",
    "PIDAntiWindupConfig",
    "QpMixerConfig",
    "DEFAULT_PD_CONFIG",
    "DEFAULT_PID_CONFIG",
    "DEFAULT_QP_MIXER_CONFIG",
]


def __getattr__(name: str):
    if name == "PDControllerConfig":
        from .pd_controller_cfg import PDControllerConfig
        return PDControllerConfig
    if name == "PIDAntiWindupConfig":
        from .pid_anti_windup_controller_cfg import PIDAntiWindupConfig
        return PIDAntiWindupConfig
    if name == "QpMixerConfig":
        from .qp_mixer_cfg import QpMixerConfig
        return QpMixerConfig
    if name == "DEFAULT_PD_CONFIG":
        from .pd_controller_cfg import DEFAULT_PD_CONFIG
        return DEFAULT_PD_CONFIG
    if name == "DEFAULT_PID_CONFIG":
        from .pid_anti_windup_controller_cfg import DEFAULT_PID_CONFIG
        return DEFAULT_PID_CONFIG
    if name == "DEFAULT_QP_MIXER_CONFIG":
        from .qp_mixer_cfg import DEFAULT_QP_MIXER_CONFIG
        return DEFAULT_QP_MIXER_CONFIG
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
