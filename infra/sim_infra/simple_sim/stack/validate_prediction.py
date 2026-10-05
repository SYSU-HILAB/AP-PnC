"""Run in the NMPC builder: test actual CasADi model before/after C codegen."""
import json

import casadi as ca
import numpy as np

from ap_pnc.env import artifact_path, get_git_root
from ap_pnc.nmpc_gen.model import tailsitter_flu


def main():
    model = tailsitter_flu()
    assert (model.x.numel(), model.u.numel(), model.p.numel()) == (15, 4, 7)
    f = ca.Function("test_prediction", [model.x, model.u, model.p], [model.f_expl_expr])
    jx = ca.Function("test_jx", [model.x, model.u, model.p], [ca.jacobian(model.f_expl_expr, model.x)])
    ju = ca.Function("test_ju", [model.x, model.u, model.p], [ca.jacobian(model.f_expl_expr, model.u)])
    params = np.array([0., 0., 0., 0.076, 0.1, 0.12, 0.15])
    x = np.zeros(15)
    x[9] = 1
    x[13] = 9.81
    u = np.array([9.81, 0., 0., 0.])
    np.testing.assert_allclose(np.asarray(f(x, u, params)).ravel(), 0, atol=1e-12)
    x[13] = 8
    x[6:9] = [.1, -.2, .3]
    u = np.array([12., .5, -.5, 1.])
    actual = np.asarray(f(x, u, params)).ravel()
    np.testing.assert_allclose(actual[6:9], (u[1:] - x[6:9]) / params[4:])
    np.testing.assert_allclose(actual[13], (u[0] - x[13]) / params[3])
    state_jac = np.asarray(jx(x, u, params))
    command_jac = np.asarray(ju(x, u, params))
    np.testing.assert_allclose(state_jac[6:9, 6:9], -np.diag(1 / params[4:]))
    np.testing.assert_allclose(command_jac[6:9, 1:4], np.diag(1 / params[4:]))
    assert np.count_nonzero(state_jac[6:9, :6]) == 0
    assert np.count_nonzero(command_jac[6:9, :1]) == 0
    # Signed aero feedback: cz is the measured body-X coefficient, not its negation.
    wind = x.copy()
    wind[3] = 5
    wind[14] = -.1
    assert float(f(wind, u, params)[3]) == -2.5
    initial = x.copy()
    dt = .001
    for _ in range(1000):
        k1 = np.asarray(f(x, u, params)).ravel()
        k2 = np.asarray(f(x + dt*k1/2, u, params)).ravel()
        k3 = np.asarray(f(x + dt*k2/2, u, params)).ravel()
        k4 = np.asarray(f(x + dt*k3, u, params)).ravel()
        x += dt*(k1+2*k2+2*k3+k4)/6
    expected_thrust = u[0] + (initial[13] - u[0])*np.exp(-1/params[3])
    expected_rates = u[1:] + (initial[6:9] - u[1:])*np.exp(-1/params[4:])
    np.testing.assert_allclose(x[13], expected_thrust, atol=1e-9)
    np.testing.assert_allclose(x[6:9], expected_rates, atol=1e-9)
    out = artifact_path(get_git_root() / ".artifacts/vtol-matlab/prediction-validation.json")
    result = {"status": "passed", "nx": 15, "nu": 4, "np": 7,
              "states": "p,v,omega,q_wxyz,collective_specific_force,cz",
              "command": "collective_specific_force_sp,omega_sp_xyz",
              "checks": ["hover", "lag derivative", "Jacobians", "no torque coupling",
                         "signed aero coefficient", "1000-step analytic lag comparison"],
              "actuator_model_in_prediction": False}
    out.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
