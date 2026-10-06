"""Build preparation only: extract private numeric parameters, never vendor source."""

import hashlib
import json
import re
import subprocess
from decimal import Decimal

import numpy as np
import yaml
from scipy.io import loadmat, savemat

from tooling.env import artifact_path, get_git_root

NUMBER = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"


def active_source(path):
    """Read authorized local input, stripping comments without executing MATLAB."""
    return "\n".join(line.split("%", 1)[0] for line in path.read_text().splitlines())


def scalar_assignment(code, name):
    values = re.findall(rf"(?m)^\s*{re.escape(name)}\s*=\s*({NUMBER})\s*;", code)
    if not values or len({Decimal(v) for v in values}) != 1:
        raise ValueError(f"unsupported local scalar assignment: {name}")
    return Decimal(values[0])


def local_calibration(source):
    """Extract calibration only at import time; never embed private values here."""
    esc = active_source(source / "Functions/ESC.m")
    pattern = (
        rf"(?m)^\s*RPM\s*=\s*({NUMBER})\s*\*\s*\(\s*({NUMBER})"
        rf"\s*\*\s*PWM\s*\*\s*PWM\s*([+-])\s*({NUMBER})"
        rf"\s*\*\s*PWM\s*([+-])\s*({NUMBER})\s*\)\s*;"
    )
    matches = re.findall(pattern, esc)
    if len(matches) != 1:
        raise ValueError("unsupported local ESC polynomial")
    scale, quadratic, sign_linear, linear, sign_constant, constant = matches[0]
    esc_poly = [
        float(Decimal(scale) * Decimal(v))
        for v in (quadratic, sign_linear + linear, sign_constant + constant)
    ]
    controller = active_source(source / "Functions/TailsitterRateController.m")
    alpha = scalar_assignment(controller, "alpha")
    if alpha <= 0:
        raise ValueError("invalid local PID divisor")
    gains = {}
    for key in ("kp", "ki", "kd"):
        gains[key] = []
        for axis in "xyz":
            matches = re.findall(
                rf"(?m)^\s*{key.capitalize()}_{axis}\s*=\s*({NUMBER})\s*/\s*alpha\s*;", controller
            )
            if len(matches) != 1:
                raise ValueError("unsupported local PID gain")
            gains[key].append(float(matches[0]) / float(alpha))
    motor = active_source(source / "Functions/Motor_Propeller_Dynamics.m")
    matches = re.findall(r"(?m)^\s*damping_const\s*=\s*\[([^\]]+)\]\s*;", motor)
    if len(matches) != 1:
        raise ValueError("unsupported local damping vector")
    terms = [v.strip() for v in matches[0].split(";")]
    if len(terms) != 3 or not all(re.fullmatch(NUMBER, v) for v in terms):
        raise ValueError("unsupported local damping coefficients")
    ticks = scalar_assignment(
        active_source(source / "run_tailsitter_simulator.m"), "sampletime"
    ) * Decimal(10**9)
    if ticks <= 0 or ticks != ticks.to_integral_value():
        raise ValueError("invalid local controller period")
    return {
        "esc_poly": esc_poly,
        "damping_wing": [float(v) for v in terms],
        "inner_dt_ns": int(ticks),
        **gains,
    }


def main():
    root = get_git_root()
    upstream = root / ".artifacts/upstream/VTOL-SIM-MATLAB"
    commit = subprocess.check_output(
        ["git", "-c", "safe.directory=" + str(upstream), "-C", str(upstream), "rev-parse", "HEAD"],
        text=True,
    ).strip()
    assert (
        commit == "14b8114900968b1df3b3933947b3a41715491385"
    ), "upstream revision changed; review before importing"
    source = upstream / "MATLAB Simulator/tail_sitter simulator"
    mat = source / "Quadcopter Structure Files/quadModel_X_VTOL.mat"
    assert mat.is_file(), "clone the authorized private upstream under .artifacts/upstream first"
    data = loadmat(mat, simplify_cells=True)["quadModel"]
    out = artifact_path(root / ".artifacts/vtol-matlab")
    out.mkdir(parents=True, exist_ok=True)
    c = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]])
    inertia = c.T @ data["Jb"] @ c
    assert np.allclose(inertia, np.diag(np.diag(inertia)))
    profile = {
        "plant": {
            "mass": float(data["mass"]),
            "gravity": float(data["g"]),
            "inertia": np.diag(inertia).tolist(),
        },
        "actuator": {
            "model": "practical",
            "arms_m": [float(data[f"d{i}"]) for i in range(1, 5)],
            "arm_angles_rad": [float(data[f"a1{i}"]) for i in range(1, 5)],
            "tilts_rad": [float(data[f"a2{i}"]) for i in range(1, 5)],
            "mixer": data["MotorThrust_FMadjust"].astype(float).tolist(),
            "motor_tau_s": float(data["T"]),
            "diameter_m": float(data["diameter"]),
            "ct": float(data["ct"]),
            "cq": float(data["cq"]),
            "jm": float(data["Jm"]),
            "pwm_min": float(data["PWM_min"]),
            "pwm_max": float(data["PWM_max"]),
            "thrust_poly": data["FJ_p"].tolist(),
            "torque_poly": data["QJ_p"].tolist(),
            **local_calibration(source),
        },
    }
    (out / "profile.yaml").write_text(yaml.safe_dump(profile, sort_keys=False))
    numeric = {k: v for k, v in data.items() if k not in ("FJ", "QJ")}
    savemat(out / "quad_model_numeric.mat", {"quadModel": numeric})
    files = [mat, source / "run_tailsitter_simulator.m"] + [
        source / "Functions" / f
        for f in [
            "Mixing.m",
            "Mix_final_output.m",
            "PWM.m",
            "ESC.m",
            "Propellor.m",
            "Motor_Dynamics_State_Eq.m",
            "Motor_Dynamics.m",
            "Motor_Propeller_Dynamics.m",
            "FM_b.m",
            "Motor_Gyro.m",
            "Body2Motor.m",
            "xyz_to_R.m",
            "TailsitterRateController.m",
            "myPID.m",
        ]
    ]
    provenance = {
        "repository": "SYSU-HILAB/VTOL-SIM-MATLAB",
        "visibility": "private",
        "upstream_commit": commit,
        "profile_sha256": hashlib.sha256((out / "profile.yaml").read_bytes()).hexdigest(),
        "source_sha256": {
            str(p.relative_to(upstream)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files
        },
        "mass_source": "main loads quadModel_X_VTOL.mat; generator .m differs",
        "no_control_surfaces": True,
        "adaptations": [
            "wing->FLU proper rotation",
            "NMPC N/kg -> static calibrated throttle",
            "motor RPM is RK4 state; source uses Euler x=T*RPM at 4 ms",
            "initial motors statically trimmed; source IC.w4 not reused",
            "zero-RPM propeller algebraic limit; not a measured windmilling model",
        ],
        "publication": "No upstream LICENSE; keep local, no automatic public redistribution",
    }
    (out / "provenance.json").write_text(json.dumps(provenance, indent=2))
    print(out / "profile.yaml")


if __name__ == "__main__":
    main()
