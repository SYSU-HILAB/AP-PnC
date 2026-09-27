"""Fixed-step closed-loop simulation: reference x controller x plant."""

from __future__ import annotations

import numpy as np

from .controllers.base import Controller
from .plant import QuadPlant, VehicleState
from .reference import Reference

_E3 = np.array([0.0, 0.0, 1.0])


def initial_state_from_reference(ref: Reference, gravity: float = 9.81) -> VehicleState:
    """Start on the reference with the attitude the controller would command."""
    p0 = ref.p[0]
    v0 = ref.v[0]
    a0 = ref.a[0]
    yb0 = ref.yb[0]

    F_d = a0 + gravity * _E3
    b3 = F_d / np.linalg.norm(F_d)
    b2 = yb0 - float(yb0 @ b3) * b3
    n2 = np.linalg.norm(b2)
    b2 = b2 / n2 if n2 > 1e-9 else np.array([0.0, 1.0, 0.0])
    b1 = np.cross(b2, b3)
    R0 = np.column_stack([b1, b2, b3])
    return VehicleState(p=p0.copy(), v=v0.copy(), R=R0, omega=np.zeros(3))


def simulate(
    ref: Reference,
    controller: Controller,
    plant: QuadPlant,
    state: VehicleState | None = None,
    dt: float = 1e-3,
    control_dt: float | None = None,
) -> dict[str, np.ndarray]:
    """Run the closed loop from t=0 and return the logged signals.

    Args:
        ref: reference trajectory
        controller: stateful controller (reset() is called here)
        plant: rigid-body plant
        state: initial state (defaults to :func:`initial_state_from_reference`)
        dt: integration step [s]
        control_dt: controller period [s] (defaults to dt)

    The full log (including any start-up transient) is returned; use
    ``tracking_metrics(records, settle_time=...)`` to exclude the transient.
    """
    controller.reset(ref)
    if state is None:
        state = initial_state_from_reference(ref, plant.cfg.gravity)
    control_dt = control_dt or dt

    log: dict[str, list] = {
        k: []
        for k in ("t", "p", "v", "omega", "thrust", "moment", "p_ref", "v_ref")
    }

    t = 0.0
    end = ref.duration
    last_control = None
    next_control_t = 0.0
    while t <= end:
        # Refresh the control at its own rate (zero-order hold in between).
        if last_control is None or t >= next_control_t - 1e-12:
            last_control = controller.act(t, state)
            next_control_t = t + control_dt

        state = plant.step(state, last_control.thrust, last_control.moment, dt)

        p_d, v_d, _, _ = ref.sample(t)
        log["t"].append(t)
        log["p"].append(state.p.copy())
        log["v"].append(state.v.copy())
        log["omega"].append(state.omega.copy())
        log["thrust"].append(last_control.thrust)
        log["moment"].append(last_control.moment.copy())
        log["p_ref"].append(p_d.copy())
        log["v_ref"].append(v_d.copy())

        t += dt

    return {k: np.asarray(vs, dtype=float) for k, vs in log.items()}
