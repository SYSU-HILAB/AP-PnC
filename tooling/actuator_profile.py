"""Import calibrated actuator parameters once, before starting a simulation."""

import copy
import math


def import_matlab_actuator(source: dict) -> dict:
    """Return FLU parameters without mutating the authorized numeric snapshot.

    Unstamped MATLAB snapshots have an explicit damping_wing field. Reject
    ambiguous layouts rather than guessing their axes. Runtime loaders accept
    only the stamped FLU result, never either source layout interchangeably.
    """
    result = copy.deepcopy(source)
    if result.get("body_frame") == "FLU":
        if "damping_wing" in result:
            raise ValueError("FLU actuator contains incompatible parameter axes")
        return result
    if "body_frame" in result or "damping_wing" not in result:
        raise ValueError("actuator requires a stamped FLU profile or a MATLAB numeric snapshot")

    def vector(key, count):
        values = result[key]
        if not isinstance(values, list) or len(values) != count:
            raise ValueError(f"invalid actuator parameter shape: {key}")
        if not all(isinstance(v, int | float) and math.isfinite(v) for v in values):
            raise ValueError(f"invalid actuator parameter values: {key}")
        return values

    for key in ("kp", "ki", "kd", "damping_wing"):
        values = vector(key, 3)
        result["damping_body" if key == "damping_wing" else key] = [values[2], values[1], values[0]]
    del result["damping_wing"]
    result["tilts_rad"] = [-value for value in vector("tilts_rad", 4)]
    mixer = result["mixer"]
    if not isinstance(mixer, list) or len(mixer) != 4:
        raise ValueError("invalid actuator mixer shape")
    for row in mixer:
        if (
            not isinstance(row, list)
            or len(row) != 4
            or not all(isinstance(v, int | float) and math.isfinite(v) for v in row)
        ):
            raise ValueError("invalid actuator mixer row")
    result["mixer"] = [[row[0], row[3], -row[2], row[1]] for row in mixer]
    result["body_frame"] = "FLU"
    return result
