"""
Trajectory CSV loading and interpolation utilities for the lu_mpc controller.
"""

import os
from collections.abc import Sequence
from pathlib import Path

import numpy as np

from ...utils import Colors, color_print
from ...utils.math_utils import wing_frd_to_quad_flu, extract_yb_from_quat


TEMP_DATA_SUBDIR = Path("artifacts") / "simulations" / "temp_data"

REQUIRED_FIELDS: dict[tuple[str, ...], str] = {
    ("time",): "Time vector",
    ("pos_x", "pos_y", "pos_z"): "Position (3D)",
    ("vel_x", "vel_y", "vel_z"): "Velocity (3D)",
    ("yb_x", "yb_y", "yb_z"): "Body Y-axis",
    ("df_phi_alpha",): "DF phi Alpha",
    ("df_lyu_alpha",): "DF Lyu Alpha",
    ("df_phi_quat_w", "df_phi_quat_x", "df_phi_quat_y", "df_phi_quat_z"): "DF phi Quaternion",
    ("df_lyu_quat_w", "df_lyu_quat_x", "df_lyu_quat_y", "df_lyu_quat_z"): "DF Lyu Quaternion",
}

OPTIONAL_FIELDS: dict[tuple[str, ...], str] = {
    ("thrust",): "Thrust command",
    ("aero_force_z",): "Aerodynamic force Z",
    ("ap_x", "ap_y", "ap_z"): "Aerodynamic center",
    ("S_ap_x", "S_ap_z"): "Aerodynamic derivatives",
    ("B_Omega_b_y",): "Body angular rate Y",
    ("alpha_dot",): "Alpha rate of change",
    ("B_aero_force_z",): "Aero force Z (body)",
    ("quat_w", "quat_x", "quat_y", "quat_z"): "Odom quaternion (legacy)",
    ("quat_odom_w", "quat_odom_x", "quat_odom_y", "quat_odom_z"): "Odom quaternion",
}


def _repo_root() -> Path:
    """Best-effort repository root detection."""
    return Path(os.environ.get("AP_PNC_DIR", Path(__file__).resolve().parents[3]))


def _temp_data_dir() -> Path:
    return _repo_root() / TEMP_DATA_SUBDIR


def _find_planning_csv() -> Path | None:
    """Locate the latest planning CSV emitted by the global planner."""
    temp_dir = _temp_data_dir()
    if not temp_dir.exists():
        return None

    planning_files = sorted(
        (path for path in temp_dir.glob("planning_*.csv") if path.is_file()),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    if planning_files:
        return planning_files[0]

    fallback = temp_dir / "planning.csv"
    if fallback.is_file():
        return fallback
    return None


def _stack_columns(data: np.ndarray, columns: Sequence[str]) -> np.ndarray:
    """Stack named columns into a matrix."""
    names = data.dtype.names or []
    missing = [name for name in columns if name not in names]
    if missing:
        raise ValueError(f"Missing required columns in trajectory CSV: {missing}")
    stacked = np.column_stack([np.atleast_1d(data[name]) for name in columns])
    return stacked


def _format_field_group(field_names: tuple[str, ...]) -> str:
    """Return a comma-separated representation of field names."""
    return ", ".join(field_names)


def _log_required_fields(names: Sequence[str]) -> list[str]:
    """Print status of required fields and return missing column names."""
    missing: list[str] = []
    for field_names, desc in REQUIRED_FIELDS.items():
        label = _format_field_group(field_names)
        if all(field in names for field in field_names):
            color_print(
                f"  {Colors.CHECKMARK} {desc:20} ({label})",
                Colors.GREEN,
            )
        else:
            missing_fields = [field for field in field_names if field not in names]
            color_print(
                f"  {Colors.CROSS} {desc:20} ({label}) "
                f"[{Colors.BOLD}MISSING: {missing_fields}{Colors.BOLD + Colors.RED}]",
                Colors.RED,
            )
            missing.extend(missing_fields)
    return missing


def _log_optional_fields(names: Sequence[str]) -> None:
    """Print optional field availability."""
    for field_names, desc in OPTIONAL_FIELDS.items():
        label = _format_field_group(field_names)
        color = Colors.GREEN if all(field in names for field in field_names) else Colors.RESET
        status = Colors.CHECKMARK if color == Colors.GREEN else Colors.CROSS
        color_print(f"  {status} {desc:30} ({label})", color)


def load_trajectory_csv(csv_path: str) -> dict[str, np.ndarray]:
    """
    Load trajectory data from a planner-generated CSV.

    Required and optional columns are documented in ``REQUIRED_FIELDS`` and
    ``OPTIONAL_FIELDS``. Orientation columns are mandatory; legacy CSVs without
    quaternion data are rejected.
    """
    csv_path_obj = Path(csv_path)
    if not csv_path_obj.exists():
        raise FileNotFoundError(f"Trajectory CSV not found: {csv_path}")

    data = np.genfromtxt(csv_path_obj, delimiter=",", names=True)
    if data.ndim == 0:
        data = np.array([data], dtype=data.dtype)

    names = data.dtype.names or []

    color_print("Trajectory CSV Validation", Colors.BOLD + Colors.BLUE)
    color_print(f"  File: {csv_path_obj}", Colors.BLUE)
    missing_required = _log_required_fields(names)

    if missing_required:
        color_print(
            f"\n{Colors.BOLD}ERROR:{Colors.RED + Colors.BOLD} Not a valid trajectory CSV!",
            Colors.RED + Colors.BOLD,
        )
        color_print(f"  Missing required fields: {missing_required}", Colors.BOLD + Colors.RED)
        color_print(
            "  Please use a CSV generated by global_planning.cpp with full orientation data.",
            Colors.YELLOW,
        )
        raise ValueError(f"Missing required columns: {missing_required}")

    color_print("\nOptional Fields:", Colors.YELLOW)
    _log_optional_fields(names)

    # ========================================================================
    # Load Data
    # ========================================================================

    trajectory: dict[str, np.ndarray] = {}
    trajectory["time"] = np.atleast_1d(data["time"])
    trajectory["position"] = _stack_columns(data, ["pos_x", "pos_y", "pos_z"])
    trajectory["velocity"] = _stack_columns(data, ["vel_x", "vel_y", "vel_z"])
    trajectory["yb"] = _stack_columns(data, ["yb_x", "yb_y", "yb_z"])

    # Optional odometry quaternion recorded by planner (for frame alignment)
    odom_columns: list[str] | None = None
    for candidate in [
        ["quat_odom_w", "quat_odom_x", "quat_odom_y", "quat_odom_z"],
        ["quat_w", "quat_x", "quat_y", "quat_z"],
    ]:
        if all(col in names for col in candidate):
            odom_columns = candidate
            break
    if odom_columns is not None:
        trajectory["odom_quat"] = _stack_columns(data, odom_columns)

    thrust_col = "thrust" if "thrust" in names else None
    trajectory["thrust"] = (
        np.atleast_1d(data[thrust_col]) if thrust_col else np.zeros_like(trajectory["time"])
    )

    df_phi_alpha = np.atleast_1d(data["df_phi_alpha"])
    trajectory["df_phi_alpha"] = df_phi_alpha
    trajectory["alpha"] = df_phi_alpha

    df_phi_quat_columns = ["df_phi_quat_w", "df_phi_quat_x", "df_phi_quat_y", "df_phi_quat_z"]
    trajectory["df_phi_quat"] = _stack_columns(data, df_phi_quat_columns)

    # Load df_lyu_alpha (required)
    trajectory["df_lyu_alpha"] = np.atleast_1d(data["df_lyu_alpha"])

    # Load df_lyu_quat (required)
    df_lyu_quat_columns = ["df_lyu_quat_w", "df_lyu_quat_x", "df_lyu_quat_y", "df_lyu_quat_z"]
    trajectory["df_lyu_quat"] = _stack_columns(data, df_lyu_quat_columns)

    # ========================================================================
    # Summary
    # ========================================================================

    color_print(
        f"\n{Colors.CHECKMARK} Successfully loaded trajectory!", Colors.GREEN + Colors.BOLD
    )
    color_print(f"  Points:    {len(trajectory['time'])}", Colors.BLUE)
    color_print(
        f"  Time range: {trajectory['time'][0]:.2f}s → {trajectory['time'][-1]:.2f}s", Colors.BLUE
    )
    color_print(
        f"  Duration:  {trajectory['time'][-1] - trajectory['time'][0]:.2f}s", Colors.BLUE
    )
    print()

    return trajectory


class TrajectoryReference:
    """
    Trajectory reference provider with interpolation.
    Provides reference states for the NMPC controller.
    """

    def __init__(self, trajectory_data: dict[str, np.ndarray]):
        """
        Initialize trajectory reference.

        Args:
            trajectory_data: Dictionary from load_trajectory_csv()
        """
        self.time = trajectory_data["time"]
        self.position = trajectory_data["position"]
        self.velocity = trajectory_data["velocity"]
        self.thrust = trajectory_data.get("thrust", np.zeros(len(self.time)))
        self.alpha = trajectory_data.get("alpha", np.zeros(len(self.time)))
        self.df_phi_quat = trajectory_data.get("df_phi_quat")
        self.df_lyu_quat = trajectory_data.get("df_lyu_quat")
        self.odom_quat = trajectory_data.get("odom_quat")

        # Compute QUAD_FLU_FRAME quaternions from WING_FRD_FRAME
        self.quat_df_phi_quad_flu = None
        self.quat_quad_flu = None
        if self.df_phi_quat is not None:
            self.quat_df_phi_quad_flu = np.array([
                wing_frd_to_quad_flu(self.df_phi_quat[i])
                for i in range(len(self.time))
            ])
        if self.df_lyu_quat is not None:
            self.quat_quad_flu = np.array([
                wing_frd_to_quad_flu(self.df_lyu_quat[i])
                for i in range(len(self.time))
            ])


            # Fallback to CSV yb if no orientation data
        self.yb = trajectory_data.get("yb", np.zeros((len(self.time), 3)))

        # Sampling time (constant, 50Hz)
        self._sim_dt = 0.02

    def get_reference_at_time(self, t: float) -> dict[str, np.ndarray]:
        """
        Get reference state at time t using nearest index lookup (no interpolation).

        Args:
            t: Time in seconds

        Returns:
            Dictionary with reference state:
            - position: (3,) array
            - velocity: (3,) array
            - yb: (3,) array
            - alpha: scalar
            - quat_quad_flu: (4,) array [w, x, y, z] in QUAD_FLU_BODY_FRAME
            - df_phi_quat: (4,) array [w, x, y, z] in WING_FRD_BODY_FRAME 
            - df_lyu_quat: (4,) array [w, x, y, z] in WING_FRD_BODY_FRAME 
        """
        # Find nearest index (clamp to valid range, no wrap-around)
        idx = int(np.round(t / self._sim_dt))
        idx = max(0, min(idx, len(self.time) - 1))

        ref = {
            "position": self.position[idx].copy(),
            "velocity": self.velocity[idx].copy(),
            "yb": self.yb[idx].copy(),
            "alpha": float(self.alpha[idx]),
        }

        # Add quat_quad_flu as quat_quad_flu for err_state_nmpc
        if self.quat_quad_flu is not None:
            quat_quad_flu = self.quat_quad_flu[idx].copy()
            norm = np.linalg.norm(quat_quad_flu)
            if norm > 1e-6:
                quat_quad_flu = quat_quad_flu / norm
            ref["quat_quad_flu"] = quat_quad_flu

        # Keep wing frame quaternions for reference
        if self.df_phi_quat is not None:
            df_phi_quat = self.df_phi_quat[idx].copy()
            norm = np.linalg.norm(df_phi_quat)
            if norm > 1e-6:
                df_phi_quat = df_phi_quat / norm
            ref["df_phi_quat"] = df_phi_quat

        if self.df_lyu_quat is not None:
            df_lyu_quat = self.df_lyu_quat[idx].copy()
            norm = np.linalg.norm(df_lyu_quat)
            if norm > 1e-6:
                df_lyu_quat = df_lyu_quat / norm
            ref["df_lyu_quat"] = df_lyu_quat

        return ref
    

    def get_reference_window(
        self,
        cur_idx_ref: int,
        ctrl_dt: float,
        N: int,
    ) -> dict[str, np.ndarray]:
        """
        Get reference trajectory for MPC prediction horizon using index-based downsampling.

        The trajectory is sampled at sample_dt (must be 0.02s = 50Hz).
        Downsampling factor = ctrl_dt / sample_dt must be an integer.
        Reference indices: cur_idx_ref + downsample_factor * i for i in 0..N

        Args:
            cur_idx_ref: Current reference index in the trajectory
            sample_dt: Sampling time of the trajectory (must be 0.02s)
            ctrl_dt: Control time step of the MPC
            N: Number of steps in the MPC horizon

        Returns:
            Dictionary with reference arrays:
            - position: (N+1, 3) array
            - velocity: (N+1, 3) array
            - yb: (N+1, 3) array
            - quat_quad_flu: (N+1, 4) array from QUAD_FLU_BODY_FRAME if available
        """
        # Validate sample_dt
        sample_dt = 0.02  # Force to 50Hz for now

        # Calculate and validate downsampling factor
        downsample_factor = ctrl_dt / sample_dt
        if not np.isclose(downsample_factor, np.round(downsample_factor)):
            raise ValueError(
                f"downsample_factor = ctrl_dt/sample_dt must be integer. "
                f"ctrl_dt={ctrl_dt}, sample_dt={sample_dt}, factor={downsample_factor}"
            )
        downsample_factor = int(downsample_factor)

        # Total trajectory length
        total_steps = len(self.time)
        final_idx = total_steps - 1

        # If current index is past the end, set all references to final state
        if cur_idx_ref >= final_idx:
            position_ref = np.tile(self.position[final_idx], (N + 1, 1))
            velocity_ref = np.tile(self.velocity[final_idx], (N + 1, 1))
            yb_ref = np.tile(self.yb[final_idx], (N + 1, 1))
            quat_quad_flu_ref = (
                np.tile(self.quat_quad_flu[final_idx], (N + 1, 1))
                if self.quat_quad_flu is not None
                else None
            )
        else:
            # Normal case: compute indices with clamp to end
            indices = cur_idx_ref + downsample_factor * np.arange(N + 1)
            indices = np.clip(indices, 0, final_idx)
            position_ref = self.position[indices]
            velocity_ref = self.velocity[indices]
            yb_ref = self.yb[indices]
            quat_quad_flu_ref = self.quat_quad_flu[indices] if self.quat_quad_flu is not None else None

        return {
            "position": position_ref,
            "velocity": velocity_ref,
            "yb": yb_ref,
            "quat_quad_flu": quat_quad_flu_ref,
        }


def get_trajectory_path() -> str:
    """
    Get the default trajectory file path - latest planning output.

    Finds the most recently modified planning_*.csv in artifacts/simulations/temp_data/.

    Returns:
        Path to the latest planning CSV

    Raises:
        FileNotFoundError: If no planning CSV is found (user must run planner first)
    """
    planning_csv = _find_planning_csv()
    if planning_csv:
        return str(planning_csv)
    temp_dir = _temp_data_dir()
    raise FileNotFoundError(
        f"No planning CSV found in {temp_dir}. "
        f"Please run the global planner first to generate a trajectory."
    )
