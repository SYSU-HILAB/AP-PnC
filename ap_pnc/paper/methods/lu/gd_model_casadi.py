# Created on 2024-12-20
#
# Symbolic CasADi implementation for aerodynamic coefficient calculations
# Based on the JAX implementation from gd_model_jax.py
#
import dataclasses

import casadi as cs
import numpy as np

from .aero_coefficients import get_numpy_coefficients


@dataclasses.dataclass
class GdModelConfig:
    """Configuration for the symbolic aerodynamic model."""

    rho: float = 1.185  # Air density in kg/m^3

    # Coefficient arrays (same as JAX version)
    kCL: np.ndarray = None
    kCD: np.ndarray = None
    kCY: np.ndarray = None
    kCLL: np.ndarray = None
    kCm: np.ndarray = None
    kCn: np.ndarray = None

    def __post_init__(self):
        """Initialize coefficient arrays if not provided."""
        coeffs = get_numpy_coefficients()

        if self.kCL is None:
            self.kCL = coeffs.kCL.copy()

        if self.kCD is None:
            self.kCD = coeffs.kCD.copy()

        if self.kCY is None:
            self.kCY = coeffs.kCY.copy()

        if self.kCLL is None:
            self.kCLL = coeffs.kCLL.copy()

        if self.kCm is None:
            self.kCm = coeffs.kCm.copy()

        if self.kCn is None:
            self.kCn = coeffs.kCn.copy()


# Default configuration instance
DEFAULT_CONFIG = GdModelConfig()


class SymbolicGdModel:
    """Symbolic CasADi implementation of the aerodynamic model."""

    def __init__(self, config: GdModelConfig = DEFAULT_CONFIG):
        """
        Initialize the symbolic aerodynamic model.

        Args:
            config: Configuration object containing model parameters
        """
        self.config = config
        self._build_symbolic_model()

    def _logistic_func(self, alpha, alpha1, k1, alpha2, k2):
        """Symbolic logistic function."""
        P1 = 1.0 / (1 + cs.exp(k1 * (alpha - alpha1)))
        P2 = 1.0 / (1 + cs.exp(-k2 * (alpha - alpha2)))
        return 1 - P1 - P2

    def _deg2rad(self, deg):
        """Convert degrees to radians symbolically."""
        return deg * (cs.pi / 180.0)

    def _reg_cl(self, alpha, beta):
        """Symbolic regression calculation for CL coefficient."""
        sa = cs.sin(alpha)
        ca = cs.cos(alpha)
        sb = cs.sin(beta)
        cb = cs.cos(beta)
        sa2 = sa**2
        ca2 = ca**2
        sb2 = sb**2
        cb2 = cb**2
        sa3 = sa**3
        ca3 = ca**3

        comp_B = cs.vertcat(cb2, alpha * cb2)

        comp_N = cs.vertcat(
            cb2 * ca3,
            ca * sb2,
            ca * cb2 * sa2,
            sa * ca2 * cb2,
            sa * sb2,
            sa3 * cb2,
            cs.fabs(sb) * cb * ca2,
            sa * ca * cb * cs.fabs(sb),
            cs.fabs(sb) * cb * sa2,
        )

        w_B1 = self._logistic_func(
            alpha, self._deg2rad(-190), 1, self._deg2rad(-170), 1
        )
        comp_B1 = cs.vertcat(cb2, (alpha + self._deg2rad(180)) * cb2)
        reg_B1 = w_B1 * comp_B1

        w_N11 = self._logistic_func(
            alpha, self._deg2rad(-170), 1, self._deg2rad(-20), 0.5
        )
        reg_N11 = w_N11 * comp_N

        w_B2 = self._logistic_func(alpha, self._deg2rad(-11), 20, self._deg2rad(18), 10)
        reg_B2 = w_B2 * comp_B

        w_N2 = self._logistic_func(alpha, self._deg2rad(18), 10, self._deg2rad(170), 1)
        reg_N2 = w_N2 * comp_N

        w_B3 = self._logistic_func(alpha, self._deg2rad(170), 1, self._deg2rad(190), 1)
        comp_B3 = cs.vertcat(cb2, (alpha - self._deg2rad(180)) * cb2)
        reg_B3 = w_B3 * comp_B3

        reg = cs.vertcat(reg_B1, reg_N11, reg_B2, reg_N2, reg_B3)
        return reg

    def _reg_cd(self, alpha, beta):
        """Symbolic regression calculation for CD coefficient."""
        sa = cs.sin(alpha)
        ca = cs.cos(alpha)
        sb = cs.sin(beta)
        cb = cs.cos(beta)
        sa2 = sa**2
        ca2 = ca**2
        sb2 = sb**2
        cb2 = cb**2
        sa3 = sa**3
        ca3 = ca**3

        comp_B = cs.vertcat(cb2, alpha * cb2)

        comp_N = cs.vertcat(
            cb2 * ca3,
            ca * sb2,
            ca * cb2 * sa2,
            sa * ca2 * cb2,
            sa * sb2,
            sa3 * cb2,
            cs.fabs(sb) * cb * ca2,
            sa * ca * cb * cs.fabs(sb),
            cs.fabs(sb) * cb * sa2,
        )

        w_B1 = self._logistic_func(
            alpha, self._deg2rad(-190), 1, self._deg2rad(-170), 1
        )
        comp_B1 = cs.vertcat(cb2, (alpha + self._deg2rad(180)) * cb2)
        reg_B1 = w_B1 * comp_B1

        w_N1 = self._logistic_func(
            alpha, self._deg2rad(-170), 1, self._deg2rad(-10), 0.5
        )
        reg_N1 = w_N1 * comp_N

        w_B2 = self._logistic_func(alpha, self._deg2rad(-11), 20, self._deg2rad(18), 10)
        reg_B2 = w_B2 * comp_B

        w_N2 = self._logistic_func(alpha, self._deg2rad(18), 10, self._deg2rad(170), 1)
        reg_N2 = w_N2 * comp_N

        w_B3 = self._logistic_func(alpha, self._deg2rad(170), 1, self._deg2rad(190), 1)
        comp_B3 = cs.vertcat(cb2, (alpha - self._deg2rad(180)) * cb2)
        reg_B3 = w_B3 * comp_B3

        reg = cs.vertcat(reg_B1, reg_N1, reg_B2, reg_N2, reg_B3)
        return reg

    def _reg_cy(self, alpha, beta):
        """Symbolic regression calculation for CY coefficient."""
        sa = cs.sin(alpha)
        ca = cs.cos(alpha)
        sb = cs.sin(beta)
        cb = cs.cos(beta)
        sb2 = sb**2
        cb2 = cb**2
        signb = beta * cs.fabs(beta)

        comp_N = cs.vertcat(signb * sb2, sb * cb * ca, sb * cb * sa)

        w_B1 = self._logistic_func(
            alpha, self._deg2rad(-190), 1, self._deg2rad(-170), 1
        )
        comp_B1 = cs.vertcat(cb2, (alpha + self._deg2rad(180)) * cb2)
        reg_B1 = w_B1 * comp_B1

        w_N11 = self._logistic_func(
            alpha, self._deg2rad(-170), 1, self._deg2rad(-90), 1
        )
        reg_N11 = w_N11 * comp_N

        w_N12 = self._logistic_func(alpha, self._deg2rad(-90), 1, self._deg2rad(-11), 1)
        reg_N12 = w_N12 * comp_N

        w_N2 = self._logistic_func(alpha, self._deg2rad(20), 1, self._deg2rad(170), 1)
        reg_N2 = w_N2 * comp_N

        w_B3 = self._logistic_func(alpha, self._deg2rad(170), 1, self._deg2rad(190), 1)
        comp_B3 = cs.vertcat(cb2, (alpha - self._deg2rad(180)) * cb2)
        reg_B3 = w_B3 * comp_B3

        reg = cs.vertcat(reg_B1, reg_N11, reg_N12, reg_N2, reg_B3)
        return reg

    def _reg_cll(self, alpha, beta):
        """Symbolic regression calculation for Cll coefficient."""
        sa = cs.sin(alpha)
        ca = cs.cos(alpha)
        sb = cs.sin(beta)
        cb = cs.cos(beta)
        sb2 = sb**2
        cb2 = cb**2
        signb = beta * cs.fabs(beta)

        comp_B = cs.vertcat(cb2, alpha * cb2)
        comp_N = cs.vertcat(signb * sb2, sb * cb * ca, sb * cb * sa)

        w_B1 = self._logistic_func(
            alpha, self._deg2rad(-190), 1, self._deg2rad(-170), 1
        )
        comp_B1 = cs.vertcat(cb2, (alpha + self._deg2rad(180)) * cb2)
        reg_B1 = w_B1 * comp_B1

        w_N11 = self._logistic_func(
            alpha, self._deg2rad(-170), 1, self._deg2rad(-90), 5
        )
        reg_N11 = w_N11 * comp_N

        w_N12 = self._logistic_func(
            alpha, self._deg2rad(-90), 5, self._deg2rad(-20), 0.5
        )
        reg_N12 = w_N12 * comp_N

        w_B2 = self._logistic_func(alpha, self._deg2rad(-11), 20, self._deg2rad(18), 10)
        reg_B2 = w_B2 * comp_B

        w_N22 = self._logistic_func(alpha, self._deg2rad(18), 10, self._deg2rad(90), 5)
        reg_N22 = w_N22 * comp_N

        w_N2 = self._logistic_func(alpha, self._deg2rad(90), 5, self._deg2rad(170), 1)
        reg_N2 = w_N2 * comp_N

        w_B3 = self._logistic_func(alpha, self._deg2rad(170), 1, self._deg2rad(190), 1)
        comp_B3 = cs.vertcat(cb2, (alpha - self._deg2rad(180)) * cb2)
        reg_B3 = w_B3 * comp_B3

        reg = cs.vertcat(reg_B1, reg_N11, reg_N12, reg_B2, reg_N2, reg_N22, reg_B3)
        return reg

    def _reg_cm(self, alpha, beta):
        """Symbolic regression calculation for Cm coefficient."""
        sa = cs.sin(alpha)
        ca = cs.cos(alpha)
        sb = cs.sin(beta)
        cb = cs.cos(beta)
        sa2 = sa**2
        ca2 = ca**2
        sb2 = sb**2
        cb2 = cb**2

        comp_B = cs.vertcat(cb2, alpha * cb2)
        comp_N = cs.vertcat(
            cb2 * ca2,
            sb2,
            cb2 * sa2,
            cs.fabs(sb) * cb * ca,
            sa * ca * cb2,
            cs.fabs(sb) * cb * sa,
        )

        w_B1 = self._logistic_func(
            alpha, self._deg2rad(-190), 1, self._deg2rad(-170), 1
        )
        comp_B1 = cs.vertcat(cb2, (alpha + self._deg2rad(180)) * cb2)
        reg_B1 = w_B1 * comp_B1

        w_N11 = self._logistic_func(
            alpha, self._deg2rad(-170), 1, self._deg2rad(-20), 0.5
        )
        reg_N11 = w_N11 * comp_N

        w_B2 = self._logistic_func(alpha, self._deg2rad(-11), 20, self._deg2rad(18), 10)
        reg_B2 = w_B2 * comp_B

        w_N2 = self._logistic_func(alpha, self._deg2rad(18), 10, self._deg2rad(170), 1)
        reg_N2 = w_N2 * comp_N

        w_B3 = self._logistic_func(alpha, self._deg2rad(170), 1, self._deg2rad(190), 1)
        comp_B3 = cs.vertcat(cb2, (alpha - self._deg2rad(180)) * cb2)
        reg_B3 = w_B3 * comp_B3

        reg = cs.vertcat(reg_B1, reg_N11, reg_B2, reg_N2, reg_B3)
        return reg

    def _reg_cn(self, alpha, beta):
        """Symbolic regression calculation for Cn coefficient."""
        sa = cs.sin(alpha)
        ca = cs.cos(alpha)
        sb = cs.sin(beta)
        cb = cs.cos(beta)
        sb2 = sb**2
        cb2 = cb**2
        signb = beta * cs.fabs(beta)

        comp_B = cs.vertcat(cb2, alpha * cb2)
        comp_N = cs.vertcat(signb * sb2, sb * cb * ca, sb * cb * sa)

        w_B1 = self._logistic_func(
            alpha, self._deg2rad(-190), 1, self._deg2rad(-170), 1
        )
        comp_B1 = cs.vertcat(cb2, (alpha + self._deg2rad(180)) * cb2)
        reg_B1 = w_B1 * comp_B1

        w_N1 = self._logistic_func(
            alpha, self._deg2rad(-170), 1, self._deg2rad(-10), 10
        )
        reg_N1 = w_N1 * comp_N

        w_B2 = self._logistic_func(alpha, self._deg2rad(-10), 10, self._deg2rad(18), 10)
        reg_B2 = w_B2 * comp_B

        w_N2 = self._logistic_func(alpha, self._deg2rad(0), 0.5, self._deg2rad(170), 1)
        reg_N2 = w_N2 * comp_N

        w_B3 = self._logistic_func(alpha, self._deg2rad(170), 1, self._deg2rad(190), 1)
        comp_B3 = cs.vertcat(cb2, (alpha - self._deg2rad(180)) * cb2)
        reg_B3 = w_B3 * comp_B3

        reg = cs.vertcat(reg_B1, reg_N1, reg_B2, reg_N2, reg_B3)
        return reg

    def _compute_aerodynamic_coeffs(self, alpha, beta):
        """Compute all aerodynamic coefficients symbolically."""
        # Calculate regression terms
        reg_cl = self._reg_cl(alpha, beta)
        reg_cd = self._reg_cd(alpha, beta)
        reg_cy = self._reg_cy(alpha, beta)
        reg_cll = self._reg_cll(alpha, beta)
        reg_cm = self._reg_cm(alpha, beta)
        reg_cn = self._reg_cn(alpha, beta)

        # Coefficient calculation using dot product
        CL = cs.dot(reg_cl, self.config.kCL)
        CD = cs.dot(reg_cd, self.config.kCD)
        CY = cs.dot(reg_cy, self.config.kCY)
        CLL = cs.dot(reg_cll, self.config.kCLL)
        CM = cs.dot(reg_cm, self.config.kCm)
        CN = cs.dot(reg_cn, self.config.kCn)

        return CL, CD, CY, CLL, CM, CN

    def _compute_angles(self, velb):
        """Compute angle of attack and sideslip angle from body velocity."""
        vbx = velb[0]
        vby = velb[1]
        vbz = velb[2]

        # Calculation
        v_norm = cs.sqrt(vbx**2 + vby**2 + vbz**2) + 1e-8
        v_squared = v_norm**2
        alpha = cs.atan2(vbz, vbx)
        beta = cs.asin(vby / v_norm)

        return alpha, beta, v_norm, v_squared

    def _trans_stab_to_body(self, alpha, vec):
        """Transform from stability frame to body frame."""
        # Stability to body frame transformation matrix using SX symbolic expressions

        T_sb = cs.vertcat(
            cs.horzcat(cs.cos(alpha), 0, -cs.sin(alpha)),
            cs.horzcat(0, 1, 0),
            cs.horzcat(cs.sin(alpha), 0, cs.cos(alpha)),
        )

        return T_sb @ vec

    def _trans_body_to_stab(self, alpha, vec):
        """Transform from stability frame to body frame."""
        # Stability to body frame transformation matrix using SX symbolic expressions

        T_bs = cs.vertcat(
            cs.horzcat(cs.cos(alpha), 0, cs.sin(alpha)),
            cs.horzcat(0, 1, 0),
            cs.horzcat(-cs.sin(alpha), 0, cs.cos(alpha)),
        )

        return T_bs @ vec

    def _build_symbolic_model(self):
        """Build the symbolic CasADi model."""
        # Define symbolic inputs
        self.velb = cs.SX.sym("velb", 3)  # Body-frame velocity [u, v, w]

        # Compute angles
        alpha, beta, v_norm, v_squared = self._compute_angles(self.velb)

        # Compute aerodynamic coefficients
        CL, CD, CY, CLL, CM, CN = self._compute_aerodynamic_coeffs(alpha, beta)

        # Compute forces in stability frame
        rho = self.config.rho
        forces_dyl = cs.vertcat(
            -0.5 * rho * v_squared * CD,
            0.5 * rho * v_squared * CY,
            -0.5 * rho * v_squared * CL,
        )

        # Transform forces to body frame
        forces_body = self._trans_stab_to_body(alpha, forces_dyl)

        # Compute moments (already in body frame)
        moments_body = 0.5 * rho * v_squared * cs.vertcat(0.0, CM, 0.0)

        # Store symbolic expressions
        self.alpha = alpha
        self.beta = beta
        self.forces = forces_body
        self.moments = moments_body

        # Create function for evaluation
        self._func = cs.Function(
            "aerodynamics_body",
            [self.velb],
            [cs.vertcat(self.forces, self.moments), alpha, beta],
        )

    @property
    def func(self):
        """Get the CasADi function for aerodynamic evaluation."""
        return self._func

    def evaluate(self, velb):
        """
        Evaluate the aerodynamic model for given body velocity.

        Args:
            velb: Body-frame velocity [u, v, w] as numpy array (3,)

        Returns:
            Tuple of (forces, moments, alpha, beta)
            - forces: Forces in body frame [Fx, Fy, Fz] as numpy array (3,)
            - moments: Moments in body frame [Mx, My, Mz] as numpy array (3,)
            - alpha: Angle of attack in radians as float
            - beta: Sideslip angle in radians as float
        """
        forces_moments, alpha, beta = self.func(velb)
        forces_moments = forces_moments.full().flatten()
        forces = forces_moments[:3]
        moments = forces_moments[3:6]
        alpha = float(alpha)
        beta = float(beta)

        return forces, moments, alpha, beta

    def get_symbolic_expressions(self):
        """
        Get the symbolic expressions for use in other models.

        Returns:
            Dictionary containing symbolic expressions
        """
        return {
            "velb": self.velb,
            "alpha": self.alpha,
            "beta": self.beta,
            "forces": self.forces,
            "moments": self.moments,
        }

    # =========================================================================
    # Factory Classmethod
    # =========================================================================

    @classmethod
    def create(cls, config: GdModelConfig = DEFAULT_CONFIG) -> "SymbolicGdModel":
        """
        Factory classmethod to create a symbolic aerodynamic model.

        Args:
            config: Configuration object containing model parameters

        Returns:
            SymbolicGdModel instance
        """
        return cls(config)


def _create_mesh_grid(mesh_size: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Create angle grids in degrees and radians."""
    alpha_deg = np.linspace(-180, 180, mesh_size)
    beta_deg = np.linspace(-90, 90, mesh_size)
    alpha_rad = np.deg2rad(alpha_deg)
    beta_rad = np.deg2rad(beta_deg)
    alpha_grid_deg, beta_grid_deg = np.meshgrid(alpha_deg, beta_deg, indexing="ij")
    alpha_grid, beta_grid = np.meshgrid(alpha_rad, beta_rad, indexing="ij")
    return alpha_grid_deg, beta_grid_deg, alpha_grid, beta_grid


def _evaluate_mesh_point(model, alpha_val: float, beta_val: float, v_mag: float = 10.0):
    """Evaluate aerodynamic coefficients at a single mesh grid location."""
    vbx = v_mag * np.cos(alpha_val) * np.cos(beta_val)
    vby = v_mag * np.sin(beta_val)
    vbz = v_mag * np.sin(alpha_val) * np.cos(beta_val)
    velb = np.array([vbx, vby, vbz])

    forces, moments, _, _ = model.evaluate(velb)
    rho = model.config.rho
    dynamic_pressure = 0.5 * rho * v_mag**2
    forces_stability = model._trans_body_to_stab(alpha_val, forces)

    cl = -forces_stability[2] / dynamic_pressure
    cd = -forces_stability[0] / dynamic_pressure
    cy = forces_stability[1] / dynamic_pressure
    cll = moments[0] / dynamic_pressure
    cm = moments[1] / dynamic_pressure
    cn = moments[2] / dynamic_pressure
    return cl, cd, cy, cll, cm, cn


def _compute_mesh_coefficients(model, alpha_grid: np.ndarray, beta_grid: np.ndarray):
    """Compute aerodynamic coefficient meshes."""
    mesh_size = alpha_grid.shape[0]
    coeffs = {
        "CL": np.zeros_like(alpha_grid),
        "CD": np.zeros_like(alpha_grid),
        "CY": np.zeros_like(alpha_grid),
        "CLL": np.zeros_like(alpha_grid),
        "CM": np.zeros_like(alpha_grid),
        "CN": np.zeros_like(alpha_grid),
    }

    print(f"Evaluating {mesh_size}x{mesh_size} mesh points...")
    for i in range(mesh_size):
        for j in range(mesh_size):
            values = _evaluate_mesh_point(model, alpha_grid[i, j], beta_grid[i, j])
            for key, value in zip(coeffs.keys(), values, strict=False):
                coeffs[key][i, j] = value
    return coeffs


def generate_mesh_data(filename="mesh_aero_data_casadi", cache_dir=None, mesh_size=361):
    """
    Generate 3D mesh data for aerodynamic coefficients using CasADi.
    Saves data to .npy file for later plotting.

    Args:
        filename (str): Name for the data file (without extension)
        cache_dir (str, optional): Custom cache directory
        mesh_size (int): Size of the mesh grid (mesh_size x mesh_size points)

    Returns:
        dict: Dictionary containing the mesh data
    """
    print("Generating CasADi 3D mesh data...")

    # Import data utilities
    from ...plotting.data_utils import save_mesh_data

    # Create model
    model = SymbolicGdModel.create()
    alpha_deg_grid, beta_deg_grid, alpha_grid, beta_grid = _create_mesh_grid(mesh_size)

    print("Computing aerodynamic coefficients with CasADi...")
    coeffs = _compute_mesh_coefficients(model, alpha_grid, beta_grid)

    save_mesh_data(
        alpha_deg_grid,
        beta_deg_grid,
        coeffs["CL"],
        coeffs["CD"],
        coeffs["CY"],
        coeffs["CLL"],
        coeffs["CM"],
        coeffs["CN"],
        filename=filename,
        cache_dir=cache_dir,
        backend="casadi_symbolic",
    )

    print("CasADi mesh data generation completed!")
    print(
        f"Data shape: ALPHA={alpha_deg_grid.shape}, BETA={beta_deg_grid.shape}, "
        f"CL={coeffs['CL'].shape}"
    )

    return {"ALPHA_DEG": alpha_deg_grid, "BETA_DEG": beta_deg_grid, **coeffs}


def generate_cl_cd_cm_coordinated_data(
    filename="coordinated_aero_data_casadi", cache_dir=None
):
    """
    Generate CL, CD, CM coefficients data under coordinated flight assumption (beta = 0).
    Saves data to .npy file for later plotting.

    Args:
        filename (str): Name for the data file (without extension)
        cache_dir (str, optional): Custom cache directory

    Returns:
        dict: Dictionary containing the coordinated flight data
    """
    print(
        "Generating CasADi aerodynamic coefficients under coordinated flight (beta = 0)..."
    )

    # Import data utilities
    from ...plotting.data_utils import save_coordinated_data

    # Create model
    model = SymbolicGdModel.create()

    # Generate alpha range (angle of attack)
    alpha_deg = np.linspace(-30, 120, 1501)  # -30 to 120 degrees, 0.1 degree increments
    alpha_rad = alpha_deg * (np.pi / 180.0)

    # Coordinated flight assumption: beta = 0 (sideslip angle)
    np.zeros_like(alpha_rad)  # Beta = 0 for all alpha values

    print("Computing aerodynamic coefficients with CasADi...")

    # Initialize arrays
    CL = np.zeros_like(alpha_rad)
    CD = np.zeros_like(alpha_rad)
    CM = np.zeros_like(alpha_rad)

    # Evaluate model for each alpha value
    print(f"Evaluating {len(alpha_deg)} alpha values...")
    for i in range(len(alpha_deg)):
        # Create velocity from angle
        alpha_val = alpha_rad[i]

        # Set velocity magnitude (can be arbitrary, coefficients are normalized)
        v_mag = 10.0  # m/s
        vbx = v_mag * np.cos(alpha_val)  # beta = 0, so cos(beta) = 1
        vby = 0.0  # beta = 0
        vbz = v_mag * np.sin(alpha_val)

        velb = np.array([vbx, vby, vbz])

        # Evaluate model
        forces, moments, _, _ = model.evaluate(velb)

        # Extract coefficients from forces and moments
        rho = model.config.rho
        v_squared = v_mag**2
        dynamic_pressure = 0.5 * rho * v_squared

        # Convert body frame forces to stability frame before extracting coefficients
        forces_stability = model._trans_body_to_stab(alpha_val, forces)

        # Extract coefficients from stability frame forces
        CL[i] = (
            -forces_stability[2] / dynamic_pressure
        )  # Lift coefficient (negative Z direction)
        CD[i] = (
            -forces_stability[0] / dynamic_pressure
        )  # Drag coefficient (negative X direction)
        CM[i] = (
            moments[1] / dynamic_pressure
        )  # Pitching moment coefficient (body frame)

    # Save data using data utilities
    save_coordinated_data(
        alpha_deg, CL, CD, CM, filename=filename, cache_dir=cache_dir
    )

    print("CasADi coordinated flight data generation completed!")
    print(
        f"Data shape: alpha={alpha_deg.shape}, CL={CL.shape}, CD={CD.shape}, CM={CM.shape}"
    )

    # Return the data dictionary for immediate use
    return {"alpha_deg": alpha_deg, "CL": CL, "CD": CD, "CM": CM}
