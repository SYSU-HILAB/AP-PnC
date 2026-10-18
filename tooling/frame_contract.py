"""Persistent aerodynamic artifact contract: body FLU and collective +Z.

alpha=atan2(vx,vz); beta=atan2(vy,hypot(vx,vz)), positive left.
Two-output force coefficients are (Fx/V², Fz/V²), without mass normalization.
"""

FRAME_CONVENTION = "flu-alpha-xz-beta-left-v1"


def require_frame(value: str | None) -> None:
    if value != FRAME_CONVENTION:
        raise ValueError("aerodynamics data requires the stamped body-FLU frame convention")
