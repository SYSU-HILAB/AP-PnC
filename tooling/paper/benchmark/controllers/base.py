"""Controller interface for the benchmark pipeline."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from ..plant import VehicleState
from ..reference import Reference


@dataclass
class Control:
    """Actuation command: collective thrust along body z + body moment."""

    thrust: float            # [N]
    moment: np.ndarray       # (3,) [N*m]


class Controller(Protocol):
    """Stateful reference-tracking controller."""

    def reset(self, ref: Reference) -> None:
        """Bind a reference trajectory (called once before the loop)."""

    def act(self, t: float, state: VehicleState) -> Control:
        """Compute the control at time ``t`` for the current state."""
