"""Offline benchmark pipeline: planner_core -> reference -> controller -> plant -> data.

The benchmark package is the single orchestrator for trajectory-tracking
experiments. It talks to the pure C++ planning core through pybind11
(``planner_bindings``), runs a controller against a plant model, and writes
metrics/data. It does not depend on ROS.
"""
