"""
Trajectory CSV loading and interpolation utilities for the lu_mpc controller.
"""

import os
from collections.abc import Sequence
from pathlib import Path

import numpy as np

from ...utils import Colors, color_print
from ...utils.math_utils import wing_frd_to_quad_flu
from ..model.ctrl.aero_evaluator import AeroForceEvaluator

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
    ("S_ap_x", "S_ap_z"): "Aerodynamic derivatives",
    ("S_ap_x_dot", "S_ap_z_dot"): "Aerodynamic derivative rates",
    ("S_Omega_x", "S_Omega_y", "S_Omega_z"): "Spatial angular velocities",
    ("B_Omega_x_df_lyu", "B_Omega_y_df_lyu", "B_Omega_z_df_lyu"): "Body frame angular velocities (df_lyu)",
    ("B_fa_z_df_lyu",): "Body frame aerodynamic force z-component (df_lyu)",
}

OPTIONAL_FIELDS: dict[tuple[str, ...], str] = {
    ("ap_x", "ap_y", "ap_z"): "Aerodynamic center",
    ("alpha_dot",): "Alpha rate of change",
    ("B_ap_x_df_lyu",): "Body ap_x from df_lyu",
}


def _repo_root() -> Path:
    """Best-effort repository root detection."""
    return Path(os.environ.get("AP_PNC_DIR", Path(__file__).resolve().parents[3]))


def _temp_data_dir() -> Path:
    return _repo_root() / TEMP_DATA_SUBDIR


def find_planning_csv() -> Path | None:
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

    # Load spatial aerodynamic derivatives (required)
    # Create (N, 3) array with S_ap = [S_ap_x, 0.0, S_ap_z]
    s_ap_x = np.atleast_1d(data["S_ap_x"])
    s_ap_z = np.atleast_1d(data["S_ap_z"])
    s_ap_zeros = np.zeros_like(s_ap_x)
    trajectory["S_ap"] = np.column_stack([s_ap_x, s_ap_zeros, s_ap_z])

    # Load spatial aerodynamic derivative rates (required)
    # Create (N, 3) array with S_ap_dot = [S_ap_x_dot, 0.0, S_ap_z_dot]
    s_ap_x_dot = np.atleast_1d(data["S_ap_x_dot"])
    s_ap_z_dot = np.atleast_1d(data["S_ap_z_dot"])
    s_ap_dot_zeros = np.zeros_like(s_ap_x_dot)
    trajectory["S_ap_dot"] = np.column_stack([s_ap_x_dot, s_ap_dot_zeros, s_ap_z_dot])

    # Load spatial angular velocities (required)
    s_omega_columns = ["S_Omega_x", "S_Omega_y", "S_Omega_z"]
    trajectory["S_Omega"] = _stack_columns(data, s_omega_columns)

    # Load body frame angular velocities (required)
    b_omega_columns = ["B_Omega_x_df_lyu", "B_Omega_y_df_lyu", "B_Omega_z_df_lyu"]
    trajectory["B_Omega_df_lyu"] = _stack_columns(data, b_omega_columns)

    # Load body frame aerodynamic force z-component (required)
    trajectory["B_fa_z_df_lyu"] = np.atleast_1d(data["B_fa_z_df_lyu"])

    # ========================================================================
    # Load Optional Fields
    # ========================================================================

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

    Factory Methods:
        from_csv: Load trajectory from a CSV file path
    """

    def __init__(self, trajectory_source: str | Path | dict[str, np.ndarray]):
        """
        Initialize trajectory reference.

        Args:
            trajectory_source: CSV file path (str|Path) or pre-loaded trajectory dict.
                              If a path is provided, the CSV is loaded automatically.
        """
        if isinstance(trajectory_source, (str, Path)):
            trajectory_data = load_trajectory_csv(str(trajectory_source))
        else:
            trajectory_data = trajectory_source
        self.time = trajectory_data["time"]
        self.position = trajectory_data["position"]
        self.velocity = trajectory_data["velocity"]
        # df_lyu_alpha, rad
        self.alpha = trajectory_data["df_lyu_alpha"]

        # Initialize aero_evaluator with default config
        self.aero_evaluator = AeroForceEvaluator()

        self.df_phi_quat = trajectory_data["df_phi_quat"]
        self.df_lyu_quat = trajectory_data["df_lyu_quat"]

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

        self.yb = trajectory_data["yb"]

        # Required aerodynamic force fields
        self.S_ap = trajectory_data["S_ap"]  # Spatial aerodynamic derivatives [S_ap_x, 0.0, S_ap_z]
        self.S_ap_dot = trajectory_data["S_ap_dot"]  # Aerodynamic derivative rates [S_ap_x_dot, 0.0, S_ap_z_dot]
        self.S_Omega = trajectory_data["S_Omega"]  # Spatial angular velocities
        self.B_Omega_df_lyu = trajectory_data["B_Omega_df_lyu"]  # Body frame angular velocities
        self.B_fa_z_df_lyu = trajectory_data["B_fa_z_df_lyu"]  # Body frame aero force z-component

        # Compute aerodynamic force coefficients
        # kappa_a = S_ap_x * cos(alpha) - S_ap_z * sin(alpha)
        # kappa_b = S_ap_z_dot * cos(alpha) + S_ap_x_dot * sin(alpha)
        cos_alpha = np.cos(self.alpha)
        sin_alpha = np.sin(self.alpha)
        s_ap_x = self.S_ap[:, 0]
        s_ap_z = self.S_ap[:, 2]
        s_ap_x_dot = self.S_ap_dot[:, 0]
        s_ap_z_dot = self.S_ap_dot[:, 2]
        self.kappa_a = s_ap_x * cos_alpha - s_ap_z * sin_alpha
        self.kappa_b = s_ap_z_dot * cos_alpha + s_ap_x_dot * sin_alpha

        # Compute M_z_alpha: Jacobian of body frame z-force with respect to alpha
        # M_z_alpha = ∂B_fa_z/∂alpha (B_f_z alpha)
        self.M_z_alpha = self._compute_M_z_alpha() /2.0
        # self.M_z_alpha = 10.0 * np.ones_like(self.M_z_alpha)


        #<HARD override for testing
        # Compute alpha_dot: rate of change of angle of attack
        # alpha_dot = kappa_b / (M_z_alpha - kappa_a)
        ALPHA_DOT_EPS = 1e-6
        self.alpha_dot = (self.kappa_b) / ((self.M_z_alpha - self.kappa_a) + ALPHA_DOT_EPS)

        #<HARD
        #Task: plot kappa_a, kappa_b, M_z_alpha over trajectory to verify reasonable values subplots 1x3
        #Task: plot alpha alpha_dot over trajectory to verify reasonable values in a plot
        #Task: plot S_ap_x, S_ap_x_dot (they are twin); S_ap_z, S_ap_z_dot (they are twin); subplots 1x2
        #Task: add a horizontal baseline for S_ap_x_dot and S_ap_z_dot at 0.0
        #All plt.show() calls should be blocking
        #>
        # Sampling time (constant, 50Hz)
        self._sim_dt = 0.02

    def _compute_M_z_alpha(self) -> np.ndarray:
        """
        Compute M_z_alpha: Jacobian of body frame z-force with respect to alpha.

        M_z_alpha = ∂B_fa_z/∂alpha, where B_fa_z is the z-component of
        body-frame aerodynamic forces.

        Uses compute_body_forces_jacobian_alpha from aero_evaluator which
        computes ∂B_fa/∂alpha (3D vector) via automatic differentiation.

        Returns:
            M_z_alpha: (N,) array where M_z_alpha[i] = ∂B_fa_z[i]/∂alpha[i]
        """
        n_points = len(self.time)
        M_z_alpha = np.zeros(n_points)

        # Get velocity magnitude from trajectory velocity data
        velocities = self.velocity  # (N, 3) array [vx, vy, vz]
        V_magnitudes = np.linalg.norm(velocities, axis=1)  # (N,) array

        # Compute M_z_alpha for each trajectory point (only when V > 1)
        for i in range(n_points):
            V = float(V_magnitudes[i])
            if V > 1.0:
                alpha = float(self.alpha[i])
                # compute_body_forces_jacobian_alpha returns [∂B_fa_x/∂alpha, ∂B_fa_y/∂alpha, ∂B_fa_z/∂alpha]
                jacobian = self.aero_evaluator.compute_body_forces_jacobian_alpha(V, alpha)
                M_z_alpha[i] = jacobian[2]  # Extract z-component
            # else: M_z_alpha[i] remains 0.0 (already initialized)

        return M_z_alpha

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
            ctrl_dt: Control time step of the MPC
            N: Number of steps in the MPC horizon

        Returns:
            Dictionary with reference arrays:
            - position: (N+1, 3) array
            - velocity: (N+1, 3) array
            - yb: (N+1, 3) array
            - alpha: (N+1,) array
            - quat_quad_flu: (N+1, 4) array from QUAD_FLU_BODY_FRAME
            - df_phi_quat: (N+1, 4) array from WING_FRD_BODY_FRAME
            - df_lyu_quat: (N+1, 4) array from WING_FRD_BODY_FRAME
            - S_Omega: (N+1, 3) array with spatial angular velocities
            - B_Omega_df_lyu: (N+1, 3) array with body frame angular velocities
            - B_fa_z_df_lyu: (N+1,) array with body frame aerodynamic force z-component
            - S_ap: (N+1, 3) array with spatial aerodynamic derivatives [S_ap_x, 0.0, S_ap_z]
            - S_ap_dot: (N+1, 3) array with aerodynamic derivative rates [S_ap_x_dot, 0.0, S_ap_z_dot]
            - M_z_alpha: (N+1,) array with ∂B_fa_z/∂alpha (Jacobian of z-force w.r.t. alpha)
            - alpha_dot: (N+1,) array with alpha rate of change
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
            alpha_ref = np.tile(self.alpha[final_idx], N + 1)
            quat_quad_flu_ref = np.tile(self.quat_quad_flu[final_idx], (N + 1, 1))
            df_phi_quat_ref = np.tile(self.df_phi_quat[final_idx], (N + 1, 1))
            df_lyu_quat_ref = np.tile(self.df_lyu_quat[final_idx], (N + 1, 1))
            S_Omega_ref = np.tile(self.S_Omega[final_idx], (N + 1, 1))
            B_Omega_df_lyu_ref = np.tile(self.B_Omega_df_lyu[final_idx], (N + 1, 1))
            B_fa_z_df_lyu_ref = np.tile(self.B_fa_z_df_lyu[final_idx], N + 1)
            S_ap_ref = np.tile(self.S_ap[final_idx], (N + 1, 1))
            S_ap_dot_ref = np.tile(self.S_ap_dot[final_idx], (N + 1, 1))
            M_z_alpha_ref = np.tile(self.M_z_alpha[final_idx], N + 1)
            alpha_dot_ref = np.tile(self.alpha_dot[final_idx], N + 1)
        else:
            # Normal case: compute indices with clamp to end
            indices = cur_idx_ref + downsample_factor * np.arange(N + 1)
            indices = np.clip(indices, 0, final_idx)
            position_ref = self.position[indices]
            velocity_ref = self.velocity[indices]
            yb_ref = self.yb[indices]
            alpha_ref = self.alpha[indices]
            quat_quad_flu_ref = self.quat_quad_flu[indices]
            df_phi_quat_ref = self.df_phi_quat[indices]
            df_lyu_quat_ref = self.df_lyu_quat[indices]
            S_Omega_ref = self.S_Omega[indices]
            B_Omega_df_lyu_ref = self.B_Omega_df_lyu[indices]
            B_fa_z_df_lyu_ref = self.B_fa_z_df_lyu[indices]
            S_ap_ref = self.S_ap[indices]
            S_ap_dot_ref = self.S_ap_dot[indices]
            M_z_alpha_ref = self.M_z_alpha[indices]
            alpha_dot_ref = self.alpha_dot[indices]

        return {
            "position": position_ref,
            "velocity": velocity_ref,
            "yb": yb_ref,
            "alpha": alpha_ref,
            "quat_quad_flu": quat_quad_flu_ref,
            "df_phi_quat": df_phi_quat_ref,
            "df_lyu_quat": df_lyu_quat_ref,
            "S_Omega": S_Omega_ref,
            "B_Omega_df_lyu": B_Omega_df_lyu_ref,
            "B_fa_z_df_lyu": B_fa_z_df_lyu_ref,
            "S_ap": S_ap_ref,
            "S_ap_dot": S_ap_dot_ref,
            "M_z_alpha": M_z_alpha_ref,
            "alpha_dot": alpha_dot_ref,
        }


def B_Omega_lyu_quad_flu(
    B_Omega_df_lyu_ref: np.ndarray,
    alpha_dot: float | np.ndarray,
) -> np.ndarray:
    """Compute body frame angular velocity in QUAD_FLU frame.

    Transforms B_Omega_df_lyu_ref from WING_FRD_BODY_FRAME to QUAD_FLU_BODY_FRAME
    and subtracts alpha_dot from the y-component.

    The transformation:
    1. Apply wing_frd_to_quad_flu transformation (180° rotation around [1,0,1] axis)
    2. Subtract alpha_dot from the y-component (pitch rate correction)

    Args:
        B_Omega_df_lyu_ref: Angular velocity in WING_FRD_BODY_FRAME [wx, wy, wz]
        alpha_dot: Rate of change of angle of attack (scalar or array)

    Returns:
        Angular velocity in QUAD_FLU_BODY_FRAME [wx', wy', wz']
    """
    from ...utils.math_utils import WING_TO_QUAD_QUAT, quat_apply

    # Apply WING_FRD to QUAD_FLU transformation to the angular velocity vector
    B_Omega_quad_flu = quat_apply(WING_TO_QUAD_QUAT, B_Omega_df_lyu_ref)

    # Subtract alpha_dot from y-component (pitch rate correction)
    B_Omega_quad_flu[1] = B_Omega_quad_flu[1] - alpha_dot
    return B_Omega_quad_flu


def get_trajectory_path() -> str:
    """
    Get the default trajectory file path - latest planning output.

    Finds the most recently modified planning_*.csv in artifacts/simulations/temp_data/.

    Returns:
        Path to the latest planning CSV

    Raises:
        FileNotFoundError: If no planning CSV is found (user must run planner first)
    """
    planning_csv = find_planning_csv()
    if planning_csv:
        return str(planning_csv)
    temp_dir = _temp_data_dir()
    raise FileNotFoundError(
        f"No planning CSV found in {temp_dir}. "
        f"Please run the global planner first to generate a trajectory."
    )
