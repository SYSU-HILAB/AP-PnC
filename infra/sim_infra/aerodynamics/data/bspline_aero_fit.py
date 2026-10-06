# BSD 3-Clause License
#
# Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
# All rights reserved.
#
# Authors:
# Hanamy: rongerch@outlook.com
#
# Paper:
# Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.

import numpy as np
from scipy import interpolate
from scipy.optimize import minimize
import os
from pathlib import Path
import pandas as pd
from dataclasses import dataclass
from typing import Dict, List
import matplotlib.pyplot as plt
import scienceplots
from numpy.typing import NDArray

plt.style.use(['science', 'ieee'])


@dataclass
class AeroConfig:
    """Configuration for aerodynamic data fitting.

    Attributes:
        DATA_DIR: Root directory from environment variable
        DATA_PATH: Path to aerodynamic data CSV
        BSPLINE_FIT_RESULTS_DIR: Directory for fit results
        ALPHA_COL: Column name for angle of attack
        CX_ACCESS_COL: Column name for X coefficient
        CZ_ACCESS_COL: Column name for Z coefficient
        CX/CZ_COEFS/KNOTS_DUMP_KEY: Keys for storing results
        PLOT: Whether to plot results
    """
    DATA_DIR: str = os.environ.get('AP_PNC_DIR', '')
    DATA_PATH: str = 'src/aerodynamics/data/MaAero.csv'
    BSPLINE_FIT_RESULTS_DIR: str = 'src/aerodynamics/data/bspline_fit_results'
    PLOT: bool = True


    ALPHA_COL: str = 'alphaRad'
    CX_ACCESS_COL: str = 'Cx'
    CZ_ACCESS_COL: str = 'Cz'

    CX_COEFS_DUMP_KEY: str = 'cx_coefs'
    CX_KNOTS_DUMP_KEY: str = 'cx_knots'
    CZ_COEFS_DUMP_KEY: str = 'cz_coefs'
    CZ_KNOTS_DUMP_KEY: str = 'cz_knots'

    CX_BSPLINE_FIT_FILENAME: str = 'bspline_fit_cx.npz'
    CZ_BSPLINE_FIT_FILENAME: str = 'bspline_fit_cz.npz'

    @property
    def full_data_path(self) -> Path:
        """Get the full path to the data file."""
        return Path(self.DATA_DIR) / self.DATA_PATH

    @property
    def bspline_fit_results_dir(self) -> Path:
        """Get the full path to the results directory."""
        return Path(self.DATA_DIR) / self.BSPLINE_FIT_RESULTS_DIR


class BSplineFitError(Exception):
    """Custom exception for B-spline fitting operations."""
    pass


def create_basis_functions(
    knots: NDArray,
    n_basis: int,
    degree: int = 3
) -> List[interpolate.BSpline]:
    """Create B-spline basis functions.

    Args:
        knots: Knot vector for B-spline
        n_basis: Number of basis functions
        degree: Degree of B-spline

    Returns:
        List of basis spline functions
    """
    basis_functions = []
    for i in range(n_basis):
        coef = np.zeros(n_basis)
        coef[i] = 1.0
        spl = interpolate.BSpline(knots, coef, degree)
        basis_functions.append(spl)
    return basis_functions


def plot_fit_results(
    x_data: NDArray,
    y_data: NDArray,
    y_fit: NDArray,
    knots: NDArray,
    basis_functions: List[interpolate.BSpline],
    optimal_coefficients: NDArray
) -> None:
    """Plot the fitting results.

    Args:
        x_data: Independent variable data
        y_data: Dependent variable data
        y_fit: Fitted values
        knots: Knot vector
        basis_functions: List of basis functions
        optimal_coefficients: Optimal coefficients
    """
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(4, 4))

    # Top subplot: Original function and fitted curve
    ax1.plot(x_data * 180/np.pi, y_fit, '-', linewidth=0.6, label='Fitted curve')
    ax1.plot(x_data * 180/np.pi, y_data, '--', label='Data')

    # Add knot points
    unique_knots = np.unique(knots)
    knot_y_values = np.zeros_like(unique_knots)
    for coef, basis in zip(optimal_coefficients, basis_functions):
        knot_y_values += coef * basis(unique_knots)
    ax1.plot(unique_knots * 180/np.pi, knot_y_values, 'o',
            markersize=.5, label='Knot points')

    ax1.set_xlabel(r'Angle of Attack $\alpha$ (deg)')
    ax1.set_ylabel('Coefficient')

    # Bottom subplot: Basis functions
    for i, basis in enumerate(basis_functions):
        ax2.plot(x_data * 180/np.pi, basis(x_data), '-', linewidth=0.5, alpha=0.5,
                label=f'Basis {i+1}' if i==0 else '')

    ax2.set_xlabel(r'Angle of Attack $\alpha$ (deg)')
    ax2.set_ylabel('Basis Functions')

    fig.tight_layout()
    plt.show()


def fit_bspline(
    x_data: NDArray,
    y_data: NDArray,
    knots: NDArray,
    config: AeroConfig,
    degree: int = 3
) -> Dict:
    """Fit B-spline to provided data points.

    Args:
        x_data: Independent variable data points
        y_data: Dependent variable data points
        knots: Knot vector for B-spline
        config: Configuration object containing plot settings
        degree: Degree of B-spline

    Returns:
        Dictionary containing fit results

    Raises:
        BSplineFitError: If fitting fails
    """
    try:
        # Calculate number of basis functions
        n_basis = len(knots) - degree - 1
        basis_functions = create_basis_functions(knots, n_basis, degree)

        # Define objective function
        def objective(coefficients: NDArray) -> float:
            y_approx = np.zeros_like(x_data)
            for coef, basis in zip(coefficients, basis_functions):
                y_approx += coef * basis(x_data)
            return np.sum((y_approx - y_data)**2)

        # Optimize
        initial_coefficients = np.zeros(n_basis)
        result = minimize(objective, initial_coefficients, method='BFGS')

        if not result.success:
            raise BSplineFitError(f"Optimization failed: {result.message}")

        optimal_coefficients = result.x

        # Calculate fitted values
        y_fit = np.zeros_like(x_data)
        for coef, basis in zip(optimal_coefficients, basis_functions):
            y_fit += coef * basis(x_data)

        if config.PLOT:
            plot_fit_results(x_data, y_data, y_fit, knots,
                           basis_functions, optimal_coefficients)

        return {
            'coefficients': optimal_coefficients,
            'error': result.fun,
            'fitted_values': y_fit,
            'basis_functions': basis_functions
        }

    except Exception as e:
        raise BSplineFitError(f"B-spline fitting failed: {str(e)}")


def get_interior_knots(coefficient_type: str) -> NDArray:
    """Get interior knots based on coefficient type.

    Args:
        coefficient_type: Either 'cx' or 'cz'

    Returns:
        Array of interior knots

    Raises:
        ValueError: If invalid coefficient type
    """
    if coefficient_type == 'cx':
        return np.array([-155, -150, -130, -80, -60, -40, -20, -5, 0,
                        10, 20, 40, 60, 80, 100, 120, 140, 160]) * np.pi/180
    elif coefficient_type == 'cz':
        return np.array([-155, -150, -130, -80, -60, -40, -20, -5, 0,
                        10, 12, 16, 20, 25, 30, 40, 60, 80,
                        100, 130, 150, 152, 154, 170]) * np.pi/180
    else:
        raise ValueError(f"Invalid coefficient type: {coefficient_type}")


def fit_and_save_bspline(
    data: pd.DataFrame,
    coefficient_type: str,
    config: AeroConfig
) -> tuple[Dict, NDArray]:
    """Fit B-spline and save results.

    Args:
        data: DataFrame containing aerodynamic data
        coefficient_type: Either 'cx' or 'cz'
        config: Configuration object

    Returns:
        Tuple of (fit results, knots)

    Raises:
        BSplineFitError: If fitting or saving fails
    """
    try:
        x = data[config.ALPHA_COL].values
        y = data[getattr(config, f'{coefficient_type.upper()}_ACCESS_COL')].values

        # Get interior knots and create full knot vector
        interior_knots = get_interior_knots(coefficient_type)
        x_min, x_max = x.min(), x.max()
        knots = np.concatenate([
            [x_min] * 4,
            interior_knots,
            [x_max] * 4
        ])

        # Fit the data
        result = fit_bspline(x, y, knots, config, degree=3)
        print(f"Final L2 error for {coefficient_type}: {result['error']}")

        # Create results directory if needed
        results_dir = config.bspline_fit_results_dir
        results_dir.mkdir(parents=True, exist_ok=True)

        # Save results
        filename = getattr(config, f'{coefficient_type.upper()}_BSPLINE_FIT_FILENAME')
        coefs_key = getattr(config, f'{coefficient_type.upper()}_COEFS_DUMP_KEY')
        knots_key = getattr(config, f'{coefficient_type.upper()}_KNOTS_DUMP_KEY')

        np.savez(results_dir / filename,
                **{coefs_key: result['coefficients'],
                   knots_key: knots})

        return result, knots

    except Exception as e:
        raise BSplineFitError(f"Failed to fit and save {coefficient_type}: {str(e)}")


def main() -> None:
    """Main function to run the B-spline fitting process."""
    try:
        config = AeroConfig()

        if not config.DATA_DIR:
            raise BSplineFitError("AP_PNC_DIR environment variable not set")

        # Read data
        data = pd.read_csv(config.full_data_path)

        # Fit both Cx and Cz
        fit_and_save_bspline(data, 'cx', config)
        fit_and_save_bspline(data, 'cz', config)

    except Exception as e:
        print(f"Error: {str(e)}")
        exit(1)


if __name__ == "__main__":
    main()
