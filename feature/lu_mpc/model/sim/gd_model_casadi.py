# Created on 2024-12-20
#
# Symbolic CasADi implementation for aerodynamic coefficient calculations
# Based on the JAX implementation from gd_model_jax.py
#
import casadi as cs
import dataclasses
from typing import Tuple
import numpy as np


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
        if self.kCL is None:
            self.kCL = np.array([
                -2.8465, 23.1516, -0.7335, 0.1407, 0.0930, 3.8276, -0.0006, 3.1951, -0.0289, 0.6425, 0.0164, 0.0031, 0.6174, -0.1286, 0.0342, -0.5109, 0.0982, -0.0006, 0.1985, -0.0070, 0.0410, 0.0037, -4.3496, 14.1041
            ], dtype=np.float64)

        if self.kCD is None:
            self.kCD = np.array([
                -0.5908, -1.8161, -0.2045, -0.0036, -0.2861, -0.8862, -0.0590, -0.9215, 0.0130, -0.0035, 0.4958, -0.0032, 0.0762, 0.1533, -0.0009, 0.1891, 0.0159, 0.0172, 0.2510, 0.0032, -0.1227, 0.1740, 0.7917, -4.4927
            ], dtype=np.float64)

        if self.kCY is None:
            self.kCY = np.array([
                -0.0678, 0.1151, -0.1024, 0.6002, 0.5367, -0.0922, -0.8523, -0.5220, -0.1006, 0.2266, -0.0345, -0.0768, 0.0748
            ], dtype=np.float64)

        if self.kCLL is None:
            self.kCLL = np.array([
                -0.0240, 0.0172, -0.0219, -0.0085, -0.2163, -0.0143, -0.0152, -0.1874, -0.0021, -0.0065, -0.0110, 0.0029, -0.0184, -0.0028, 0.0203, -0.0158, -0.0245, -0.0068
            ], dtype=np.float64)

        if self.kCm is None:
            self.kCm = np.array([
                -1.9963, -2.5059, 0.5987, 0.0120, 0.7977, 0.0543, 0.6310, -0.1122, -0.0798, -0.2177, -0.1059, 0.0029, -0.1388, 0.0132, -0.2605, 0.0015, -0.5789, 0.7902
            ], dtype=np.float64)

        if self.kCn is None:
            self.kCn = np.array([
                0.0681, -0.0338, 0.0054, 0.0028, -0.0052, 0.0034, -0.0048, 0.0217, 0.0111, 0.0121, 0.0685, 0.0240
            ], dtype=np.float64)


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
        sa2 = sa ** 2
        ca2 = ca ** 2
        sb2 = sb ** 2
        cb2 = cb ** 2
        sa3 = sa ** 3
        ca3 = ca ** 3

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

        w_B1 = self._logistic_func(alpha, self._deg2rad(-190), 1, self._deg2rad(-170), 1)
        comp_B1 = cs.vertcat(cb2, (alpha + self._deg2rad(180)) * cb2)
        reg_B1 = w_B1 * comp_B1

        w_N11 = self._logistic_func(alpha, self._deg2rad(-170), 1, self._deg2rad(-20), 0.5)
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
        sa2 = sa ** 2
        ca2 = ca ** 2
        sb2 = sb ** 2
        cb2 = cb ** 2
        sa3 = sa ** 3
        ca3 = ca ** 3

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

        w_B1 = self._logistic_func(alpha, self._deg2rad(-190), 1, self._deg2rad(-170), 1)
        comp_B1 = cs.vertcat(cb2, (alpha + self._deg2rad(180)) * cb2)
        reg_B1 = w_B1 * comp_B1

        w_N1 = self._logistic_func(alpha, self._deg2rad(-170), 1, self._deg2rad(-10), 0.5)
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
        sb2 = sb ** 2
        cb2 = cb ** 2
        signb = beta * cs.fabs(beta)

        comp_N = cs.vertcat(signb * sb2, sb * cb * ca, sb * cb * sa)

        w_B1 = self._logistic_func(alpha, self._deg2rad(-190), 1, self._deg2rad(-170), 1)
        comp_B1 = cs.vertcat(cb2, (alpha + self._deg2rad(180)) * cb2)
        reg_B1 = w_B1 * comp_B1

        w_N11 = self._logistic_func(alpha, self._deg2rad(-170), 1, self._deg2rad(-90), 1)
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
        sb2 = sb ** 2
        cb2 = cb ** 2
        signb = beta * cs.fabs(beta)

        comp_B = cs.vertcat(cb2, alpha * cb2)
        comp_N = cs.vertcat(signb * sb2, sb * cb * ca, sb * cb * sa)

        w_B1 = self._logistic_func(alpha, self._deg2rad(-190), 1, self._deg2rad(-170), 1)
        comp_B1 = cs.vertcat(cb2, (alpha + self._deg2rad(180)) * cb2)
        reg_B1 = w_B1 * comp_B1

        w_N11 = self._logistic_func(alpha, self._deg2rad(-170), 1, self._deg2rad(-90), 5)
        reg_N11 = w_N11 * comp_N

        w_N12 = self._logistic_func(alpha, self._deg2rad(-90), 5, self._deg2rad(-20), 0.5)
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
        sa2 = sa ** 2
        ca2 = ca ** 2
        sb2 = sb ** 2
        cb2 = cb ** 2

        comp_B = cs.vertcat(cb2, alpha * cb2)
        comp_N = cs.vertcat(cb2 * ca2, sb2, cb2 * sa2, cs.fabs(sb) * cb * ca, sa * ca * cb2, cs.fabs(sb) * cb * sa)

        w_B1 = self._logistic_func(alpha, self._deg2rad(-190), 1, self._deg2rad(-170), 1)
        comp_B1 = cs.vertcat(cb2, (alpha + self._deg2rad(180)) * cb2)
        reg_B1 = w_B1 * comp_B1

        w_N11 = self._logistic_func(alpha, self._deg2rad(-170), 1, self._deg2rad(-20), 0.5)
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
        sb2 = sb ** 2
        cb2 = cb ** 2
        signb = beta * cs.fabs(beta)

        comp_B = cs.vertcat(cb2, alpha * cb2)
        comp_N = cs.vertcat(signb * sb2, sb * cb * ca, sb * cb * sa)

        w_B1 = self._logistic_func(alpha, self._deg2rad(-190), 1, self._deg2rad(-170), 1)
        comp_B1 = cs.vertcat(cb2, (alpha + self._deg2rad(180)) * cb2)
        reg_B1 = w_B1 * comp_B1

        w_N1 = self._logistic_func(alpha, self._deg2rad(-170), 1, self._deg2rad(-10), 10)
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
        v_squared = v_norm ** 2
        alpha = cs.atan2(vbz, vbx)
        beta = cs.asin(vby / v_norm)

        return alpha, beta, v_norm, v_squared

    def _trans_stab_to_body(self, alpha, vec):
        """Transform from stability frame to body frame."""
        # Stability to body frame transformation matrix using SX symbolic expressions

        T_sb = cs.vertcat(
            cs.horzcat(cs.cos(alpha), 0 , -cs.sin(alpha)),
            cs.horzcat(0, 1, 0),
            cs.horzcat(cs.sin(alpha), 0 , cs.cos(alpha))
        )


        return T_sb @ vec

    def _trans_body_to_stab(self, alpha, vec):
        """Transform from stability frame to body frame."""
        # Stability to body frame transformation matrix using SX symbolic expressions

        T_bs = cs.vertcat(
            cs.horzcat(cs.cos(alpha), 0 , cs.sin(alpha)),
            cs.horzcat(0, 1, 0),
            cs.horzcat(-cs.sin(alpha), 0 , cs.cos(alpha))
        )


        return T_bs @ vec
    
    def _build_symbolic_model(self):
        """Build the symbolic CasADi model."""
        # Define symbolic inputs
        self.velb = cs.SX.sym('velb', 3)  # Body-frame velocity [u, v, w]

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
        moments_body = cs.vertcat(
            # 0.5 * rho * v_squared * CLL,
            cs.SX.zeros(CLL.shape),
            0.5 * rho * v_squared * CM,
            cs.SX.zeros(CN.shape),
        )

        # Store symbolic expressions
        self.alpha = alpha
        self.beta = beta
        self.forces = forces_body
        self.moments = moments_body

        # Create function for evaluation
        self._func = cs.Function(
            'aerodynamics_body',
            [self.velb],
            [cs.vertcat(self.forces, self.moments), alpha, beta]
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
            'velb': self.velb,
            'alpha': self.alpha,
            'beta': self.beta,
            'forces': self.forces,
            'moments': self.moments
        }


def create_symbolic_gd_model(config: GdModelConfig = DEFAULT_CONFIG) -> SymbolicGdModel:
    """
    Factory function to create a symbolic aerodynamic model.

    Args:
        config: Configuration object containing model parameters

    Returns:
        SymbolicGdModel instance
    """
    return SymbolicGdModel(config)


def test_symbolic_model():
    """Test the symbolic aerodynamic model."""
    print("Testing symbolic aerodynamic model...")

    # Create model
    model = create_symbolic_gd_model()

    # Test with some velocities
    test_velocities = [
        [10.0, 0.0, 0.0],      # Level flight
        [10.0, 0.0, 10.0],     # Climbing
        [10.0 * .707, 10.0, 10.0 * .707],  # Combined
    ]

    for i, velb in enumerate(test_velocities):
        print(f"\nTest case {i+1}: velb = {velb}")

        forces, moments, alpha, beta = model.evaluate(velb)

        print(f"Forces: {forces}")
        print(f"Moments: {moments}")
        print(f"Alpha: {alpha:.4f} rad ({alpha * 180/np.pi:.2f} deg)")
        print(f"Beta: {beta:.4f} rad ({beta * 180/np.pi:.2f} deg)")

    print("\nSymbolic model test completed!")


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
    from .plotting.data_utils import save_mesh_data

    # Create model
    model = create_symbolic_gd_model()

    # Generate ranges for alpha and beta
    alpha_deg = np.linspace(-180, 180, mesh_size)
    beta_deg = np.linspace(-90, 90, mesh_size)

    # Convert to radians
    alpha_rad = alpha_deg * (np.pi / 180.0)
    beta_rad = beta_deg * (np.pi / 180.0)

    # Create meshgrid
    ALPHA_DEG, BETA_DEG = np.meshgrid(alpha_deg, beta_deg, indexing='ij')
    ALPHA, BETA = np.meshgrid(alpha_rad, beta_rad, indexing='ij')

    print("Computing aerodynamic coefficients with CasADi...")

    # Initialize arrays for coefficients
    CL = np.zeros_like(ALPHA)
    CD = np.zeros_like(ALPHA)
    CY = np.zeros_like(ALPHA)
    CLL = np.zeros_like(ALPHA)
    CM = np.zeros_like(ALPHA)
    CN = np.zeros_like(ALPHA)

    # Evaluate model for each point in the mesh
    # Note: CasADi evaluation is point-wise, not batched like JAX
    print(f"Evaluating {mesh_size}x{mesh_size} mesh points...")
    for i in range(mesh_size):
        for j in range(mesh_size):
            # Create velocity from angles
            alpha_val = ALPHA[i, j]
            beta_val = BETA[i, j]

            # Set velocity magnitude (can be arbitrary, coefficients are normalized)
            v_mag = 10.0  # m/s
            vbx = v_mag * np.cos(alpha_val) * np.cos(beta_val)
            vby = v_mag * np.sin(beta_val)
            vbz = v_mag * np.sin(alpha_val) * np.cos(beta_val)

            velb = np.array([vbx, vby, vbz])

            # Evaluate model
            forces, moments, _, _ = model.evaluate(velb)

            # Extract coefficients from forces and moments
            # F = 0.5 * rho * V^2 * S * C  =>  C = F / (0.5 * rho * V^2 * S)
            # For simplicity, assuming unit reference area S = 1
            rho = model.config.rho
            v_squared = v_mag ** 2
            dynamic_pressure = 0.5 * rho * v_squared

            # Convert body frame forces to stability frame before extracting coefficients
            forces_stability = model._trans_body_to_stab(alpha_val, forces)

            # Extract coefficients from stability frame forces
            CL[i, j] = -forces_stability[2] / dynamic_pressure  # Lift coefficient (negative Z direction)
            CD[i, j] = -forces_stability[0] / dynamic_pressure  # Drag coefficient (negative X direction)
            CY[i, j] = forces_stability[1] / dynamic_pressure   # Side force coefficient (positive Y direction)

            # Moments are already coefficients (assuming unit reference)
            CLL[i, j] = moments[0] / dynamic_pressure
            CM[i, j] = moments[1] / dynamic_pressure
            CN[i, j] = moments[2] / dynamic_pressure

    print("Converting results to numpy for saving...")

    # Save data using data utilities
    data_path = save_mesh_data(
        ALPHA_DEG, BETA_DEG, CL, CD, CY, CLL, CM, CN,
        filename=filename,
        cache_dir=cache_dir,
        backend="casadi_symbolic"
    )

    print(f"CasADi mesh data generation completed!")
    print(f"Data shape: ALPHA={ALPHA_DEG.shape}, BETA={BETA_DEG.shape}, CL={CL.shape}")

    # Return the data dictionary for immediate use
    return {
        'ALPHA_DEG': ALPHA_DEG,
        'BETA_DEG': BETA_DEG,
        'CL': CL,
        'CD': CD,
        'CY': CY,
        'CLL': CLL,
        'CM': CM,
        'CN': CN
    }


def generate_cl_cd_cm_coordinated_data(filename="coordinated_aero_data_casadi", cache_dir=None):
    """
    Generate CL, CD, CM coefficients data under coordinated flight assumption (beta = 0).
    Saves data to .npy file for later plotting.

    Args:
        filename (str): Name for the data file (without extension)
        cache_dir (str, optional): Custom cache directory

    Returns:
        dict: Dictionary containing the coordinated flight data
    """
    print("Generating CasADi aerodynamic coefficients under coordinated flight (beta = 0)...")

    # Import data utilities
    from .plotting.data_utils import save_coordinated_data

    # Create model
    model = create_symbolic_gd_model()

    # Generate alpha range (angle of attack)
    alpha_deg = np.linspace(-30, 120, 1501)  # -30 to 120 degrees, 0.1 degree increments
    alpha_rad = alpha_deg * (np.pi / 180.0)

    # Coordinated flight assumption: beta = 0 (sideslip angle)
    beta = np.zeros_like(alpha_rad)  # Beta = 0 for all alpha values

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
        v_squared = v_mag ** 2
        dynamic_pressure = 0.5 * rho * v_squared

        # Convert body frame forces to stability frame before extracting coefficients
        forces_stability = model._trans_body_to_stab(alpha_val, forces)

        # Extract coefficients from stability frame forces
        CL[i] = -forces_stability[2] / dynamic_pressure  # Lift coefficient (negative Z direction)
        CD[i] = -forces_stability[0] / dynamic_pressure  # Drag coefficient (negative X direction)
        CM[i] = moments[1] / dynamic_pressure           # Pitching moment coefficient (body frame)

    # Save data using data utilities
    data_path = save_coordinated_data(
        alpha_deg, CL, CD, CM,
        filename=filename,
        cache_dir=cache_dir
    )

    print(f"CasADi coordinated flight data generation completed!")
    print(f"Data shape: alpha={alpha_deg.shape}, CL={CL.shape}, CD={CD.shape}, CM={CM.shape}")

    # Return the data dictionary for immediate use
    return {
        'alpha_deg': alpha_deg,
        'CL': CL,
        'CD': CD,
        'CM': CM
    }


def test_frame_conversion():
    """Test that body-to-stability frame coefficient conversion is working correctly."""
    print("Testing body-to-stability frame coefficient conversion...")

    # Create model
    model = create_symbolic_gd_model()

    # Test case: alpha = 10 degrees, beta = 0 degrees (coordinated flight)
    alpha_deg = 10.0
    alpha_rad = np.deg2rad(alpha_deg)
    v_mag = 20.0  # m/s

    # Create velocity vector for coordinated flight
    vbx = v_mag * np.cos(alpha_rad)
    vby = 0.0
    vbz = v_mag * np.sin(alpha_rad)
    velb = np.array([vbx, vby, vbz])

    # Evaluate model
    forces, moments, alpha, beta = model.evaluate(velb)

    print(f"Test conditions:")
    print(f"  Alpha: {alpha_deg:.1f}° ({alpha:.3f} rad)")
    print(f"  Beta: {np.rad2deg(beta):.1f}° ({beta:.3f} rad)")
    print(f"  Velocity: {v_mag:.1f} m/s")
    print(f"  Velocity vector: [{vbx:.2f}, {vby:.2f}, {vbz:.2f}]")
    print()

    # Convert to stability frame
    forces_stability = model._trans_body_to_stab(alpha, forces)

    # Calculate dynamic pressure
    rho = model.config.rho
    dynamic_pressure = 0.5 * rho * v_mag**2

    print(f"Dynamic pressure: {dynamic_pressure:.2f} Pa")
    print()

    # Extract coefficients from body frame (INCORRECT way)
    CL_body = -forces[2] / dynamic_pressure
    CD_body = -forces[0] / dynamic_pressure
    CY_body = forces[1] / dynamic_pressure

    # Extract coefficients from stability frame (CORRECT way)
    CL_stability = float(-forces_stability[2] / dynamic_pressure)
    CD_stability = float(-forces_stability[0] / dynamic_pressure)
    CY_stability = float(forces_stability[1] / dynamic_pressure)

    print("Force coefficients comparison:")
    print(f"  Body frame extraction (INCORRECT):")
    print(f"    CL_body = {CL_body:.4f}")
    print(f"    CD_body = {CD_body:.4f}")
    print(f"    CY_body = {CY_body:.4f}")
    print()
    print(f"  Stability frame extraction (CORRECT):")
    print(f"    CL_stability = {CL_stability:.4f}")
    print(f"    CD_stability = {CD_stability:.4f}")
    print(f"    CY_stability = {CY_stability:.4f}")
    print()

    # For coordinated flight (beta = 0), CY should be approximately 0
    # The difference shows the importance of proper frame conversion
    print("Validation:")
    print(f"  CY_stability (should be ~0 for beta=0): {CY_stability:.6f}")
    print(f"  CL_body vs CL_stability difference: {abs(CL_body - CL_stability):.6f}")
    print(f"  CD_body vs CD_stability difference: {abs(CD_body - CD_stability):.6f}")
    print()

    # Test with non-zero beta to show more dramatic difference
    print("Testing with non-zero beta (15 degrees):")
    beta_deg = 15.0
    beta_rad = np.deg2rad(beta_deg)

    # Create velocity vector with sideslip
    vbx = v_mag * np.cos(alpha_rad) * np.cos(beta_rad)
    vby = v_mag * np.sin(beta_rad)
    vbz = v_mag * np.sin(alpha_rad) * np.cos(beta_rad)
    velb_beta = np.array([vbx, vby, vbz])

    forces_beta, _, alpha_beta, beta_beta = model.evaluate(velb_beta)
    forces_stability_beta = model._trans_body_to_stab(alpha_beta, forces_beta)

    # Extract coefficients both ways
    CL_body_beta = -forces_beta[2] / dynamic_pressure
    CD_body_beta = -forces_beta[0] / dynamic_pressure
    CY_body_beta = forces_beta[1] / dynamic_pressure

    CL_stability_beta = float(-forces_stability_beta[2] / dynamic_pressure)
    CD_stability_beta = float(-forces_stability_beta[0] / dynamic_pressure)
    CY_stability_beta = float(forces_stability_beta[1] / dynamic_pressure)

    print(f"  Beta: {beta_deg:.1f}°")
    print(f"    CL difference: {abs(float(CL_body_beta - CL_stability_beta)):.6f}")
    print(f"    CD difference: {abs(float(CD_body_beta - CD_stability_beta)):.6f}")
    print(f"    CY difference: {abs(float(CY_body_beta - CY_stability_beta)):.6f}")
    print()

    print("✅ Frame conversion test completed!")


if __name__ == "__main__":
    test_symbolic_model()
    print()
    test_frame_conversion()