"""Main-frame C/C++/FD/kinematics validation; no upstream MATLAB claim."""

import ctypes
import json
import os
import subprocess
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from tooling.env import artifact_path, get_git_root


def main():
    out = artifact_path(get_git_root() / ".artifacts/flatness" / f"linux-{os.uname().machine}")
    fn = ctypes.CDLL(str(out / "libflatness.so")).ap_pnc_flatness_flu
    fn.argtypes = [np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")] * 6
    fn.restype = ctypes.c_int
    params = np.array([-9.81, -0.09408, 0.5, 0.05])
    seed = np.arange(1.0, 14.0) / 13

    def evaluate(z):
        v, jac, grad = np.zeros(13), np.zeros(117), np.zeros(9)
        assert fn(np.ascontiguousarray(z), params, seed, v, jac, grad) == 0
        assert np.isfinite(v).all() and np.isfinite(jac).all() and np.isfinite(grad).all()
        jac = jac.reshape(13, 9, order="F")
        np.testing.assert_allclose(grad, jac.T @ seed, atol=1e-9, rtol=1e-9)
        r = v[4:].reshape(3, 3)
        np.testing.assert_allclose(r.T @ r, np.eye(3), atol=1e-10)
        assert abs(np.linalg.det(r) - 1) < 1e-10
        return v, jac, grad

    rng = np.random.default_rng(71)
    cases = [rng.normal(size=9) * [8, 8, 3, 2, 2, 2, 3, 3, 3] for _ in range(500)]
    for speed in [0.0, 1e-12, 0.1, 0.499999, 0.500001, 1.0, 10.0]:
        cases.append(np.array([speed, 0, 0, 0, 0, 0, 0.1, 0.2, 0.3]))
    cases += [
        np.array([0, 0, 0, 0, 0, -9.81, 0, 0, 0.0]),
        np.array([1, 0, 0, 1, 0, -9.81, 0.2, 0.1, 0]),
        np.array([0, 0, 0, 1, 0, -9.81, 0.2, 0.1, 0]),
    ]
    payload = "\n".join(" ".join(format(x, ".17g") for x in z) for z in cases) + "\n"
    oracle = subprocess.run(
        [str(out / "flu_golden")], input=payload, text=True, capture_output=True, check=True
    )
    golden = np.loadtxt(oracle.stdout.splitlines())
    expected = []
    cpp_error = fd_error = kinematic_error = balance_error = 0.0
    for i, z in enumerate(cases):
        value, jac, gradient = evaluate(z)
        expected.append((value, jac, gradient))
        np.testing.assert_allclose(value, golden[i], atol=1e-9, rtol=1e-9)
        cpp_error = max(cpp_error, float(np.max(np.abs(value - golden[i]))))
        if i < 100:
            eps = 1e-5
            fd = np.column_stack(
                [
                    (evaluate(z + np.eye(9)[c] * eps)[0] - evaluate(z - np.eye(9)[c] * eps)[0])
                    / (2 * eps)
                    for c in range(9)
                ]
            )
            np.testing.assert_allclose(jac, fd, atol=2e-5, rtol=2e-5)
            fd_error = max(fd_error, float(np.max(np.abs(jac - fd))))
            direction = np.r_[z[3:6], z[6:9], np.zeros(3)]
            dr = (
                evaluate(z + eps * direction)[0][4:].reshape(3, 3)
                - evaluate(z - eps * direction)[0][4:].reshape(3, 3)
            ) / (2 * eps)
            r = value[4:].reshape(3, 3)
            omega = value[1:4]
            skew = np.array(
                [[0, -omega[2], omega[1]], [omega[2], 0, -omega[0]], [-omega[1], omega[0], 0]]
            )
            err = float(np.max(np.abs(dr - r @ skew)))
            assert err < 2e-5
            kinematic_error = max(kinematic_error, err)
            aero_x = params[1] * np.linalg.norm(z[:3]) * float(r[:, 0] @ z[:3])
            eta = z[3:6] - [0, 0, params[0]]
            err = float(np.linalg.norm(eta - r @ np.array([aero_x, 0, value[0]])))
            assert err < 1e-9
            balance_error = max(balance_error, err)
    hover = evaluate(np.zeros(9))[0]
    np.testing.assert_allclose(hover[:4], [9.81, 0, 0, 0], atol=1e-12)
    np.testing.assert_allclose(hover[4:].reshape(3, 3), np.eye(3), atol=1e-12)
    # World yaw covariance in nominal branch; fallback deliberately anchors a world heading.
    angle = 0.7
    q = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    for z in cases[:100]:
        value = evaluate(z)[0]
        rotated = evaluate(np.r_[q @ z[:3], q @ z[3:6], q @ z[6:9]])[0]
        np.testing.assert_allclose(rotated[:4], value[:4], atol=1e-9)
        np.testing.assert_allclose(
            rotated[4:].reshape(3, 3), q @ value[4:].reshape(3, 3), atol=1e-9
        )
    with ThreadPoolExecutor(max_workers=8) as pool:
        actual = list(pool.map(evaluate, cases * 4))
    for i, result in enumerate(actual):
        for a, b in zip(result, expected[i % len(cases)], strict=True):
            np.testing.assert_array_equal(a, b)
    report = {
        "status": "passed",
        "frame": "ENU/FLU, +Z thrust; no legacy adapter",
        "cpp_cases": len(cases),
        "finite_difference_cases": 100,
        "kinematic_cases": 100,
        "world_yaw_covariance_cases": 100,
        "parallel_calls": len(actual),
        "cpp_max_abs_error": cpp_error,
        "jacobian_fd_max_abs_error": fd_error,
        "rotation_kinematic_max_abs_error": kinematic_error,
        "force_balance_max_error": balance_error,
        "scope": "new physical reference ABI; legacy optimizer not silently migrated; branchwise guards",
    }
    (out / "flu-validation.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
