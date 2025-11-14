import sqlite3
from pathlib import Path
from typing import Optional
from dataclasses import dataclass

from phi_aero_writer import PhiAeroWriterConfig, phi_coefs_values, update_database_for_phi
from bspline_aero_writer import (
    BSplineAeroWriterConfig,
    read_bspline_fit_results,
    update_database_for_bspline,
    check_bspline_fit_exists,
    BSplineFitError
)
from advanced_lift_drag_writer import (
    AdvancedLiftDragWriterConfig,
    update_database_for_advanced_lift_drag,
    AdvancedLiftDragError
)

@dataclass
class AssemblyError(Exception):
    """Custom exception for assembly operations."""
    message: str
    phi_error: Optional[Exception] = None
    bspline_error: Optional[Exception] = None
    advanced_lift_drag_error: Optional[Exception] = None

def update_database_for_all(
    phi_config: PhiAeroWriterConfig,
    bspline_config: BSplineAeroWriterConfig,
    advanced_lift_drag_config: AdvancedLiftDragWriterConfig
) -> None:
    """Update all aerodynamics configurations in the database.

    Args:
        phi_config: Configuration object for phi coefficients
        bspline_config: Configuration object for B-spline coefficients
        advanced_lift_drag_config: Configuration object for advanced lift drag parameters

    Raises:
        AssemblyError: If any update operation fails
    """
    try:
        # Update phi aero configuration
        update_database_for_phi(phi_config)

        # Update bspline configuration
        update_database_for_bspline(bspline_config)

        # Update advanced lift drag configuration
        update_database_for_advanced_lift_drag(advanced_lift_drag_config)

        print("Database updated successfully with all aerodynamics configurations.")

    except Exception as e:
        raise AssemblyError(
            message="Failed to update database with coefficients",
            phi_error=e if "phi" in str(e).lower() else None,
            bspline_error=e if "spline" in str(e).lower() else None,
            advanced_lift_drag_error=e if "lift drag" in str(e).lower() else None
        )

def main() -> None:
    """Main function to run the assembly process."""
    try:
        # Initialize configs
        phi_config = PhiAeroWriterConfig()
        bspline_config = BSplineAeroWriterConfig()
        advanced_lift_drag_config = AdvancedLiftDragWriterConfig()

        # Set phi coefficients
        phi_config.phi_coefs_values = phi_coefs_values

        # Only read bspline fit results if they exist
        if not check_bspline_fit_exists(bspline_config):
            raise BSplineFitError("B-spline fit results files not found")

        read_bspline_fit_results(bspline_config)

        # Verify we have valid data
        if (len(bspline_config.cx_coefs_values) == 0 or
            len(bspline_config.cz_coefs_values) == 0):
            raise BSplineFitError("B-spline fit results are empty")

        update_database_for_all(phi_config, bspline_config, advanced_lift_drag_config)

    except (BSplineFitError, AssemblyError, AdvancedLiftDragError) as e:
        print(f"Error: {str(e)}")
        exit(1)

if __name__ == "__main__":
    main()
