"""Public synthetic fixtures; private upstream calibration must stay local."""

import importlib.util

import pytest

from tooling.env import get_git_root


@pytest.fixture
def importer():
    path = get_git_root() / "infra/sim_infra/simple_sim/stack/import_matlab.py"
    spec = importlib.util.spec_from_file_location("matlab_import_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_local_calibration_uses_input_not_embedded_values(importer, tmp_path):
    functions = tmp_path / "Functions"
    functions.mkdir()
    (functions / "ESC.m").write_text("RPM = 2*(3 * PWM * PWM + 4*PWM - 5);\n")
    controller = "% alpha = 999;\nalpha = 2;\nalpha = 2;\n"
    for name in ("Kp", "Ki", "Kd"):
        for value, axis in enumerate("xyz", start=3):
            controller += f"{name}_{axis} = {value}/alpha; % ignored comment\n"
    (functions / "TailsitterRateController.m").write_text(controller)
    (functions / "Motor_Propeller_Dynamics.m").write_text("damping_const = [-1;-2;-3];\n")
    (tmp_path / "run_tailsitter_simulator.m").write_text("sampletime = 0.01;\n")
    result = importer.local_calibration(tmp_path)
    assert result == {
        "esc_poly": [6.0, 8.0, -10.0],
        "damping_wing": [-1.0, -2.0, -3.0],
        "inner_dt_ns": 10000000,
        "kp": [1.5, 2.0, 2.5],
        "ki": [1.5, 2.0, 2.5],
        "kd": [1.5, 2.0, 2.5],
    }


def test_local_calibration_rejects_executable_expression(importer, tmp_path):
    functions = tmp_path / "Functions"
    functions.mkdir()
    (functions / "ESC.m").write_text("RPM = system('untrusted');\n")
    with pytest.raises(ValueError, match="unsupported local ESC"):
        importer.local_calibration(tmp_path)


def test_scalar_assignment_rejects_ambiguous_input(importer):
    with pytest.raises(ValueError, match="unsupported local scalar"):
        importer.scalar_assignment("alpha = 2;\nalpha = 3;\n", "alpha")
