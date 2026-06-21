from acados_template import AcadosModel

from ap_pnc.lu_mpc.model.ctrl.err_state_model import export_error_state_model


def test_export_error_state_model_dimensions_and_metadata():
    model = export_error_state_model()

    assert isinstance(model, AcadosModel)
    assert model.name == "err_state_model_lu"
    assert model.x.size()[0] == 9  # dp, dv, dtheta
    assert model.u.size()[0] == 4  # daT and domega
    assert model.p.size()[0] == 28  # v_d(3), R_d(9), aT_d(1), omega_d(3), f_a_d(3), par_f_a_par_v_b(9)

    # Dynamics expression should produce a 9x1 vector when evaluated
    assert model.f_expl_expr.shape[0] == 9
