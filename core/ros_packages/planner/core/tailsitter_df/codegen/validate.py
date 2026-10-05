"""Container-only numerical comparison; also checks concurrent C workspace use."""

import ctypes
import json
import os
import subprocess
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from ap_pnc.env import artifact_path, get_git_root


def main():
    root = get_git_root()
    out = artifact_path(root / ".artifacts" / "flatness" / f"linux-{os.uname().machine}")
    library = ctypes.CDLL(str(out / "libflatness.so"))
    fn = library.ap_pnc_flatness
    ptr = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
    fn.argtypes = [ptr] * 6
    fn.restype = ctypes.c_int
    vjp_fn = library.ap_pnc_flatness_backward
    vjp_fn.argtypes = [ptr] * 5
    vjp_fn.restype = ctypes.c_int
    params = np.array([-9.8, -0.09408, 0.5, 0.05], dtype=np.float64)
    seed = np.array([0.5, -1.0, 2.0, 0.7], dtype=np.float64)

    def evaluate(z):
        z = np.ascontiguousarray(z, dtype=np.float64)
        value, jac, backward = np.zeros(4), np.zeros(36), np.zeros(9)
        assert fn(z, params, seed, value, jac, backward) == 0
        assert np.isfinite(value).all() and np.isfinite(jac).all() and np.isfinite(backward).all()
        jac = jac.reshape((4, 9), order="F")
        np.testing.assert_allclose(backward, jac.T @ seed, rtol=1e-11, atol=1e-11)
        direct_value, direct_vjp = np.zeros(4), np.zeros(9)
        assert vjp_fn(z, params, seed, direct_value, direct_vjp) == 0
        np.testing.assert_allclose(direct_value, value, rtol=1e-12, atol=1e-12)
        np.testing.assert_allclose(direct_vjp, backward, rtol=1e-12, atol=1e-12)
        return value, jac, backward

    rng = np.random.default_rng(42)
    cases = []
    while len(cases) < 500:
        z = rng.normal(size=9) * np.array([8, 8, 3, 2, 2, 2, 3, 3, 3])
        if np.linalg.norm(z[:3]) > 1:
            cases.append(z)
    # Both special branches, plus points approaching either side of guards.
    for speed in [0, 1e-12, 0.1, 0.499999, 0.500001, 1, 10]:
        cases.append(np.array([speed, 0, 0, 1, 0, -9.8, 0.2, 0.1, 0]))
    for projected in [0, 0.049999, 0.050001, 0.2]:
        cases.append(np.array([5, 0, 0, 1, projected, -9.8, 0.2, 0.1, 0]))
    payload = "\n".join(" ".join(format(x, ".17g") for x in z) for z in cases) + "\n"
    proc = subprocess.run(
        [str(out / "flatness_golden")], input=payload, text=True, capture_output=True, check=True
    )
    golden = np.loadtxt(proc.stdout.splitlines())
    assert golden.shape == (len(cases), 40)
    value_error = jac_error = fd_error = 0.0
    expected = []
    for i, z in enumerate(cases):
        value, jac, backward = evaluate(z)
        gv = golden[i, :4]
        gj = golden[i, 4:].reshape((4, 9), order="F")
        np.testing.assert_allclose(value, gv, rtol=1e-9, atol=1e-9)
        np.testing.assert_allclose(jac, gj, rtol=1e-8, atol=1e-8)
        value_error = max(value_error, float(np.max(np.abs(value - gv))))
        jac_error = max(jac_error, float(np.max(np.abs(jac - gj))))
        expected.append((value, jac, backward))
        if i < 100:  # Away from switching surfaces: classical derivatives exist.
            numerical = np.column_stack(
                [
                    (evaluate(z + np.eye(9)[k] * 1e-5)[0] - evaluate(z - np.eye(9)[k] * 1e-5)[0])
                    / 2e-5
                    for k in range(9)
                ]
            )
            np.testing.assert_allclose(jac, numerical, rtol=2e-5, atol=2e-5)
            fd_error = max(fd_error, float(np.max(np.abs(jac - numerical))))
    # ctypes releases the GIL: simultaneous calls must not share scratch storage.
    with ThreadPoolExecutor(max_workers=8) as pool:
        actual = list(pool.map(evaluate, cases * 4))
    for i, result in enumerate(actual):
        for a, b in zip(result, expected[i % len(cases)], strict=True):
            np.testing.assert_array_equal(a, b)
    report = {
        "status": "passed",
        "golden_cases": len(cases),
        "finite_difference_cases": 100,
        "parallel_calls": len(actual),
        "parallel_workers": 8,
        "direct_vjp_without_dense_jacobian": "passed",
        "value_max_abs_error": value_error,
        "jacobian_max_abs_error": jac_error,
        "finite_difference_max_abs_error": fd_error,
        "boundary_policy": "branchwise golden comparison; no claim of smooth switching surfaces",
        "runtime": "C + libm; no Python/CasADi runtime",
    }
    (out / "validation.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
