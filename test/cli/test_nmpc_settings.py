"""YAML-only tuning and rejection of stale first-order configuration."""

from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from tooling.nmpc_gen.create_ocp import load_nmpc_settings


def test_yaml_is_the_only_tuning_source():
    settings = load_nmpc_settings()
    assert settings["objective"]["body_y"] == [52.5, 52.5, 126.0]
    assert settings["objective"]["command_derivative"] == [0.03] * 4
    assert set(settings["command_bounds"]) == {"lower", "upper"}
    # The NMPC is mass-normalized: no mass may reappear in its config.
    assert "mass" not in settings
    assert "thrust_tau_s" not in settings and "rate_tau_s" not in settings


@pytest.mark.parametrize(
    "case",
    [
        "stale_tau",
        "stale_objective",
        "zero_regularization",
        "nonfinite_weight",
        "wrong_size",
        "bad_bounds",
        "bad_grid",
        "slow_control",
    ],
)
def test_invalid_or_unused_settings_rejected(tmp_path, case):
    settings = deepcopy(load_nmpc_settings())
    if case == "stale_tau":
        settings["thrust_tau_s"] = 0.076
    elif case == "stale_objective":
        settings["objective"]["angular_velocity"] = [0, 0, 0]
    elif case == "zero_regularization":
        settings["objective"]["command_derivative"][0] = 0
    elif case == "nonfinite_weight":
        settings["objective"]["velocity"][1] = float("nan")
    elif case == "wrong_size":
        settings["objective"]["position"] = [1, 2]
    elif case == "bad_bounds":
        settings["command_bounds"]["lower"][0] = 23
    elif case == "slow_control":
        settings["ctrl_frq"] = 5.0
    else:
        settings["horizon_s"] = 1.05
    path = tmp_path / "nmpc.yaml"
    path.write_text(yaml.safe_dump({"nmpc": settings}))
    with pytest.raises(ValueError):
        load_nmpc_settings(path)


def test_relative_settings_path_rejected():
    with pytest.raises(ValueError, match="absolute"):
        load_nmpc_settings(Path("nmpc.yaml"))
