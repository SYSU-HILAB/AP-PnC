import numpy as np
from scipy import interpolate, signal
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import os
import pandas as pd
from dataclasses import dataclass
import pickle
import scienceplots
plt.style.use(['science','ieee'])

@dataclass
class AeroConfig:
    """Configuration for aerodynamic data fitting"""
    DATA_DIR: str = os.environ.get('TAILSITTER_PLANNING_DIR', '')
    DATA_PATH: str = 'src/aerodynamics/data/MaAero.csv'
    BSPLINE_FIT_RESULTS_DIR: str = 'src/aerodynamics/data/bspline_fit_results'
    ALPHA_COL: str = 'alphaRad'
    CX_ACCESS_COL: str = 'Cx'
    CX_COEFS_DUMP_KEY: str = 'cx_coefs'
    CX_KNOTS_DUMP_KEY: str = 'cx_knots'
    CX_BSPLINE_FIT_FILENAME: str = 'bspline_fit_cx.npz'
    CZ_ACCESS_COL: str = 'Cz'
    CZ_COEFS_DUMP_KEY: str = 'cz_coefs'
    CZ_KNOTS_DUMP_KEY: str = 'cz_knots'
    CZ_BSPLINE_FIT_FILENAME: str = 'bspline_fit_cz.npz'
    @property
    def full_data_path(self) -> str:
        return os.path.join(self.DATA_DIR, self.DATA_PATH)  
    
    @property
    def bspline_fit_results_dir(self) -> str:
        return os.path.join(self.DATA_DIR, self.BSPLINE_FIT_RESULTS_DIR)

# Global configuration
CONFIG = AeroConfig()



def fit_bspline(x_data, y_data, knots, degree=3, plot=True):
    """
    Fit B-spline to provided data points using specified knots.
    
    Parameters:
    -----------
    x_data : array-like
        Independent variable data points
    y_data : array-like
        Dependent variable data points
    knots : array-like
        Knot vector for B-spline
    degree : int, optional
        Degree of B-spline (default: 3)
    plot : bool, optional
        Whether to plot the results (default: True)
        
    Returns:
    --------
    dict
        Dictionary containing:
        - 'coefficients': Optimal B-spline coefficients
        - 'error': Final L2 error
        - 'fitted_values': Fitted y values
        - 'basis_functions': List of basis functions
    """
    
    # Calculate number of basis functions
    n_basis = len(knots) - degree - 1
    
    # Create basis splines
    basis_functions = []
    for i in range(n_basis):
        coef = np.zeros(n_basis)
        coef[i] = 1.0
        spl = interpolate.BSpline(knots, coef, degree)
        basis_functions.append(spl)
    
    # Define objective function for optimization
    def objective(coefficients):
        y_approx = np.zeros_like(x_data)
        for coef, basis in zip(coefficients, basis_functions):
            y_approx += coef * basis(x_data)
        return np.sum((y_approx - y_data)**2)
    
    # Optimize
    initial_coefficients = np.zeros(n_basis)
    result = minimize(objective, initial_coefficients, method='BFGS')
    optimal_coefficients = result.x
    
    # Calculate fitted values
    y_fit = np.zeros_like(x_data)
    for coef, basis in zip(optimal_coefficients, basis_functions):
        y_fit += coef * basis(x_data)
    
    if plot:
        # Create figure with two subplots
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(4, 4))
        
        # Top subplot: Original function   and fitted curve
        ax1.plot(x_data * 180/np.pi, y_fit, '-', linewidth=0.6, label='Fitted curve')
        ax1.plot(x_data * 180/np.pi, y_data, '--', label='Data')
        
        # Add knot points on the fitted curve
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
        
        # Adjust layout
        fig.tight_layout()
        plt.show()
    
    return {
        'coefficients': optimal_coefficients,
        'error': result.fun,
        'fitted_values': y_fit,
        'basis_functions': basis_functions
    }

def fit_cx_bspline_and_save(data, plot=False):
    x = data[CONFIG.ALPHA_COL].values
    y = data[CONFIG.CX_ACCESS_COL].values
    
    # Define knots (adjust these based on your data range)
    # Make sure knots span the range of your x values
    interior_knots = np.array([-155, -150, -130,
                              -80, -60, -40,
                              -20, -5, 0,
                              10, 20, 40,
                              60, 80, 100,
                              120, 140, 160]) * np.pi/180
    
    x_min, x_max = x.min(), x.max()
    knots = np.concatenate([
        [x_min] * 4,  # repeated knots at the start
        interior_knots,  # interior knots at 10 deg increments
        [x_max] * 4   # repeated knots at the end
    ])
    
    # Fit the data
    result = fit_bspline(x, y, knots, degree=3, plot=plot)
    
    print("Final L2 error:", result['error'])
    
    # if the directory does not exist, create it
    if not os.path.exists(CONFIG.bspline_fit_results_dir):
        os.makedirs(CONFIG.bspline_fit_results_dir)
    
    # Save coefficients and knots to a single file
    np.savez(os.path.join(CONFIG.bspline_fit_results_dir, CONFIG.CX_BSPLINE_FIT_FILENAME),
             **{CONFIG.CX_COEFS_DUMP_KEY: result['coefficients'],
                CONFIG.CX_KNOTS_DUMP_KEY: knots})
    
    return result, knots

def fit_cz_bspline_and_save(data, plot=False):
    x = data[CONFIG.ALPHA_COL].values
    y = data[CONFIG.CZ_ACCESS_COL].values
    
    # Define knots (adjust these based on your data range)
    # Make sure knots span the range of your x values
    interior_knots = np.array([-155, -150, -130,
                              -80, -60, -40,
                              -20, -5, 0,
                              10, 12, 16,
                              20, 25, 30,
                              40, 60, 80,
                              100, 130, 150,
                              152, 154, 170]) * np.pi/180
    
    x_min, x_max = x.min(), x.max()
    knots = np.concatenate([
        [x_min] * 4,  # repeated knots at the start
        interior_knots,  # interior knots at 10 deg increments
        [x_max] * 4   # repeated knots at the end
    ])
    
    # Fit the data
    result = fit_bspline(x, y, knots, degree=3, plot=plot)
    
    print("Final L2 error:", result['error'])
    
    # if the directory does not exist, create it
    if not os.path.exists(CONFIG.bspline_fit_results_dir):
        os.makedirs(CONFIG.bspline_fit_results_dir)
    
    # Save coefficients and knots to a single file
    np.savez(os.path.join(CONFIG.bspline_fit_results_dir, CONFIG.CZ_BSPLINE_FIT_FILENAME),
             **{CONFIG.CZ_COEFS_DUMP_KEY: result['coefficients'],
                CONFIG.CZ_KNOTS_DUMP_KEY: knots})
    
    return result, knots



# Example usage:
if __name__ == "__main__":
    # Verify configuration
    if not CONFIG.DATA_DIR:
        raise ValueError("TAILSITTER_PLANNING_DIR environment variable not set")
    
    # Read data from CSV file
    data = pd.read_csv(CONFIG.full_data_path)
    
    # Fit both Cx and Cz
    fit_cx_bspline_and_save(data)
    fit_cz_bspline_and_save(data)
