from acados_template import AcadosModel

from feature.lu_mpc.model.ctrl.err_state_model import export_error_state_model


def test_export_error_state_model_dimensions_and_metadata():
    model = export_error_state_model()

    assert isinstance(model, AcadosModel)
    assert model.name == "enmpc_lu"
    assert model.x.size()[0] == 9  # dp, dv, dtheta
    assert model.u.size()[0] == 4  # daT and domega
    assert model.p.size()[0] == 19  # p_d, v_d, R_d(9 entries), aT_d, omega_d

    # Dynamics expression should produce a 9x1 vector when evaluated
    assert model.f_expl_expr.shape[0] == 9
