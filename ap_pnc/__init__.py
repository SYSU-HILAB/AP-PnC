"""AP-PnC Python: core algorithm service (nmpc_gen) + paper (benchmark/figure)."""

import sys
from pathlib import Path

# The aerodynamics ML service lives with the C++ aero package under sim_infra;
# make it importable as a top-level `aerodynamics` package.
_AERO_PY = Path(__file__).resolve().parents[1] / "infra" / "sim_infra" / "aerodynamics" / "python"
if _AERO_PY.is_dir() and str(_AERO_PY) not in sys.path:
    sys.path.insert(0, str(_AERO_PY))
