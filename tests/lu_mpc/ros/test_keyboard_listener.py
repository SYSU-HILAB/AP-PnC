# -*- coding: future_fstrings -*-
from feature.lu_mpc.ros.keyboard_listener import is_space_keypress


def test_space_keypress_detects_space():
    assert is_space_keypress(" ")


def test_space_keypress_ignores_other_keys():
    assert not is_space_keypress("a")
    assert not is_space_keypress("")
