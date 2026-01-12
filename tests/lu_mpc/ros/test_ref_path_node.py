# -*- coding: future_fstrings -*-
import pytest

from feature.lu_mpc.ros.ref_path_node import advance_playback_time


def test_advance_without_loop_stops_at_end():
    next_time, wrapped = advance_playback_time(0.9, 0.2, 0.0, 1.0, loop_playback=False)
    assert next_time == pytest.approx(1.0)
    assert wrapped is False


def test_advance_with_loop_wraps_to_start():
    next_time, wrapped = advance_playback_time(0.9, 0.2, 0.0, 1.0, loop_playback=True)
    assert next_time == pytest.approx(0.1)
    assert wrapped is True


def test_advance_internal_step():
    next_time, wrapped = advance_playback_time(0.2, 0.1, 0.0, 1.0, loop_playback=True)
    assert next_time == pytest.approx(0.3)
    assert wrapped is False
