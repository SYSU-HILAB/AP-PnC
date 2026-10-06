"""Builder-side tests of the actual symbolic dynamics and generated OCP."""

import json

import casadi as ca
import numpy as np

from tooling.env import artifact_path, get_git_root
from tooling.nmpc_gen.create_ocp import load_nmpc_settings, ocp_nls
from tooling.nmpc_gen.model import create_tailsitter_model, rotation_matrix_from_quaternion


def main():
    model = create_tailsitter_model()
    assert (model.x.numel(), model.u.numel(), model.p.numel()) == (15, 4, 1)
    f = ca.Function("test_prediction", [model.x, model.u, model.p], [model.f_expl_expr])
    jx = ca.Function(
        "test_jx", [model.x, model.u, model.p], [ca.jacobian(model.f_expl_expr, model.x)]
    )
    ju = ca.Function(
        "test_ju", [model.x, model.u, model.p], [ca.jacobian(model.f_expl_expr, model.u)]
    )
    parameters = np.zeros(1)
    state = np.zeros(15)
    state[9] = 1.0
    state[13] = 9.81
    np.testing.assert_allclose(np.asarray(f(state, np.zeros(4), parameters)).ravel(), 0, atol=1e-12)
    state[3] = 2.0  # stay away from the existing aero approximation's norm guard
    state[13] = 8.0
    state[6:9] = [0.1, -0.2, 0.3]
    command_derivative = np.array([1.2, 0.5, -0.5, 1.0])
    actual = np.asarray(f(state, command_derivative, parameters)).ravel()
    np.testing.assert_allclose(actual[6:9], command_derivative[1:])
    np.testing.assert_allclose(actual[13], command_derivative[0])
    state_jacobian = np.asarray(jx(state, command_derivative, parameters))
    input_jacobian = np.asarray(ju(state, command_derivative, parameters))
    np.testing.assert_allclose(state_jacobian[[13, 6, 7, 8]], 0)
    np.testing.assert_allclose(input_jacobian[[13, 6, 7, 8]], np.eye(4))
    # Lumped body-X coefficient: it is signed and mass-normalized, not a wing Z term.
    signed_aero = state.copy()
    signed_aero[3] = 5.0
    signed_aero[14] = -0.1
    assert float(f(signed_aero, command_derivative, parameters)[3]) == -2.5
    initial = state.copy()
    dt = 0.001
    for _ in range(1000):
        k1 = np.asarray(f(state, command_derivative, parameters)).ravel()
        k2 = np.asarray(f(state + dt * k1 / 2, command_derivative, parameters)).ravel()
        k3 = np.asarray(f(state + dt * k2 / 2, command_derivative, parameters)).ravel()
        k4 = np.asarray(f(state + dt * k3, command_derivative, parameters)).ravel()
        state += dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6
    np.testing.assert_allclose(
        state[[13, 6, 7, 8]], initial[[13, 6, 7, 8]] + command_derivative, atol=1e-11
    )

    # Verify the renamed quaternion operators retain qdot/Rdot semantics.
    quaternion = model.x[9:13]
    rotation = rotation_matrix_from_quaternion(quaternion)
    rotation_rate = ca.reshape(
        ca.jacobian(ca.reshape(rotation, 9, 1), quaternion) @ model.f_expl_expr[9:13], 3, 3
    )
    rotation_function = ca.Function(
        "rotation_kinematics", [model.x, model.u, model.p], [rotation, rotation_rate]
    )
    rng = np.random.default_rng(17)
    for _ in range(100):
        sample = initial.copy()
        sample[9:13] = rng.normal(size=4)
        sample[9:13] /= np.linalg.norm(sample[9:13])
        sample[6:9] = rng.normal(size=3)
        wx, wy, wz = sample[6:9]
        hat = np.array([[0, -wz, wy], [wz, 0, -wx], [-wy, wx, 0]])
        r, dr = rotation_function(sample, command_derivative, parameters)
        np.testing.assert_allclose(dr, np.asarray(r) @ hat, atol=1e-12)

    ocp = ocp_nls()
    settings = load_nmpc_settings()
    assert ocp.model.cost_y_expr.numel() == 13 and ocp.model.cost_y_expr_e.numel() == 9
    output = ca.Function("cost_output", [ocp.model.x, ocp.model.u], [ocp.model.cost_y_expr])
    changed_commands = initial.copy()
    changed_commands[6:9] = [1.0, -1.0, 0.8]
    changed_commands[13] = 16.0
    np.testing.assert_allclose(output(initial, np.zeros(4)), output(changed_commands, np.zeros(4)))
    np.testing.assert_allclose(
        np.asarray(output(initial, command_derivative)).ravel()[9:], command_derivative
    )
    np.testing.assert_array_equal(ocp.constraints.idxbx, [13, 6, 7, 8])
    np.testing.assert_array_equal(ocp.constraints.idxbx_e, [13, 6, 7, 8])
    np.testing.assert_allclose(ocp.constraints.lbx, settings["command_bounds"]["lower"])
    np.testing.assert_allclose(ocp.constraints.ubx_e, settings["command_bounds"]["upper"])
    assert ocp.constraints.lbu.size == 0 and ocp.constraints.ubu.size == 0
    np.testing.assert_allclose(
        np.diag(ocp.cost.W_0)[9:], settings["objective"]["command_derivative"]
    )
    result = {
        "status": "passed",
        "abi": 3,
        "nx": 15,
        "nu": 4,
        "np": 1,
        "ny": 13,
        "nyn": 9,
        "prediction": "command_integrator",
        "checks": [
            "hover with zero derivative",
            "integrator derivatives/Jacobians",
            "1000-step linear command integration",
            "100 quaternion Rdot cases",
            "no absolute command cost",
            "stage0 derivative regularization",
            "command state bounds including terminal",
            "no stale derivative bounds",
        ],
        "actuator_model_in_prediction": False,
        "aerodynamic_coefficient_evolution": "unchanged; separate kinematics audit pending",
    }
    out = artifact_path(
        get_git_root() / ".artifacts/benchmark/nmpc-command-integrator/prediction-validation.json"
    )
    out.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
