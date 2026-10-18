"""Public binding semantics; run against the native Linux extension."""

import numpy as np
import pytest

from aerodynamics.frames import FRAME_CONVENTION

bindings = pytest.importorskip("aerodynamics_bindings")


@pytest.fixture(params=["lyu", "bspline", "phi", "advanced_lift_drag", "non_aerodynamics"])
def model(request):
    assert bindings.FRAME_CONVENTION == FRAME_CONVENTION
    return getattr(bindings, f"create_{request.param}_model")()


def test_wrench_angles_and_coefficients_use_the_same_body_axes(model):
    for alpha in np.linspace(-np.pi, np.pi, 19):
        speed = 12.0
        velocity = speed * np.array([np.sin(alpha), 0, np.cos(alpha)])
        force, moment, actual_alpha, beta = model.compute_wrench(velocity)
        assert np.isfinite(moment).all()
        assert actual_alpha == pytest.approx(alpha)
        assert beta == 0
        np.testing.assert_allclose(
            model.compute_coefficients(alpha, speed), force[[0, 2]] / speed**2
        )
    for velocity in ([1, 2, 3], [-1, -2, -3], [0, 2, 0], [0, -2, 0], [0, 0, 0]):
        _, _, alpha, beta = model.compute_wrench(velocity)
        assert alpha == pytest.approx(np.arctan2(velocity[0], velocity[2]))
        assert beta == pytest.approx(np.arctan2(velocity[1], np.hypot(velocity[0], velocity[2])))


def test_binding_generation_saves_native_parquet(tmp_path):
    from aerodynamics.data.generation import assemble_parquet_dataset_from_bindings

    output = tmp_path / "generated.parquet"
    dataset = assemble_parquet_dataset_from_bindings(num_points=9, output_path=output)
    assert len(dataset.df) == 36
    assert dataset.df["frame_convention"].eq(FRAME_CONVENTION).all()
    assert output.is_file()


def test_binding_rejects_invalid_inputs(model):
    for speed in (0, -1, np.nan, np.inf):
        with pytest.raises(ValueError):
            model.compute_coefficients(0, speed)
    with pytest.raises(ValueError):
        model.compute_coefficients(np.nan)
    with pytest.raises(ValueError, match="nonfinite"):
        model.compute_wrench([np.nan, 0, 1])
