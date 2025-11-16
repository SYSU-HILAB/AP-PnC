import sqlite3
import os
from dataclasses import dataclass
from typing import List, Optional
from pathlib import Path
import numpy as np
from numpy.typing import NDArray

@dataclass
class BSplineAeroWriterConfig:
    """Configuration for B-spline aerodynamics data writing.

    Attributes:
        ROOT_DIR: Environment variable name for root directory
        SUB_FOLDER: Subfolder path for aerodynamics data
        DB_FILENAME: SQLite database filename
        BSPLINE_AERO_TABLE_NAME: Table name for B-spline data
        CX_BSPLINE_FIT_RESULTS_PATH: Path to CX B-spline fit results
        CZ_BSPLINE_FIT_RESULTS_PATH: Path to CZ B-spline fit results
        CX_KNOTS_DUMP_KEY: Key for CX knots in data files
        CZ_KNOTS_DUMP_KEY: Key for CZ knots in data files
        CX_COEFS_DUMP_KEY: Key for CX coefficients in data files
        CZ_COEFS_DUMP_KEY: Key for CZ coefficients in data files
        SCALE_FACTOR_DUMP_KEY: Key for scale factor in database
        SCALE_FACTOR_VALUE: Computed scale factor value
    """
    ROOT_DIR: str = 'AP_PNC_DIR'
    SUB_FOLDER: str = 'src/aerodynamics'
    DB_FILENAME: str = 'aero_sim_params.db'
    BSPLINE_AERO_TABLE_NAME: str = 'bspline_aero_config'

    CX_BSPLINE_FIT_RESULTS_PATH: str = 'data/bspline_fit_results/bspline_fit_cx.npz'
    CZ_BSPLINE_FIT_RESULTS_PATH: str = 'data/bspline_fit_results/bspline_fit_cz.npz'

    CX_KNOTS_DUMP_KEY: str = 'cx_knots'
    CZ_KNOTS_DUMP_KEY: str = 'cz_knots'
    CX_COEFS_DUMP_KEY: str = 'cx_coefs'
    CZ_COEFS_DUMP_KEY: str = 'cz_coefs'

    SCALE_FACTOR_DUMP_KEY: str = 'scale_factor'
    SCALE_FACTOR_VALUE: float = 0.5 * 1.184 * 0.24 / 2.0 * 3 * 0.5483863461851688 * 1.177888074834827

    cx_coefs_values: NDArray = np.array([])
    cz_coefs_values: NDArray = np.array([])
    cx_knots_values: NDArray = np.array([])
    cz_knots_values: NDArray = np.array([])

def to_csv(data_list: NDArray) -> str:
    """Convert numpy array to CSV string format.

    Args:
        data_list: Numpy array to convert

    Returns:
        Comma-separated string of values
    """
    return ','.join(map(str, data_list))

class BSplineFitError(Exception):
    """Custom exception for B-spline fit related errors."""
    pass

def get_root_path(config: BSplineAeroWriterConfig) -> Path:
    """Get the root path from environment variable.

    Args:
        config: Configuration object containing ROOT_DIR

    Returns:
        Path object for root directory

    Raises:
        BSplineFitError: If environment variable is not set
    """
    root_dir = os.getenv(config.ROOT_DIR)
    if not root_dir:
        raise BSplineFitError(f"Environment variable {config.ROOT_DIR} is not set")
    return Path(root_dir)

def read_bspline_fit_results(config: BSplineAeroWriterConfig) -> None:
    """Read B-spline fit results and populate config object.

    Args:
        config: Configuration object to populate with fit results

    Raises:
        BSplineFitError: If files are missing or have invalid format
    """
    try:
        root_path = get_root_path(config)
        sub_folder = root_path / config.SUB_FOLDER

        cx_path = sub_folder / config.CX_BSPLINE_FIT_RESULTS_PATH
        cz_path = sub_folder / config.CZ_BSPLINE_FIT_RESULTS_PATH

        if not cx_path.exists() or not cz_path.exists():
            raise BSplineFitError(f"Missing fit results files at {cx_path} or {cz_path}")

        cx_data = np.load(cx_path)
        cz_data = np.load(cz_path)

        # Validate data keys
        for data, keys, prefix in [
            (cx_data, [config.CX_KNOTS_DUMP_KEY, config.CX_COEFS_DUMP_KEY], 'CX'),
            (cz_data, [config.CZ_KNOTS_DUMP_KEY, config.CZ_COEFS_DUMP_KEY], 'CZ')
        ]:
            if not all(key in data.files for key in keys):
                raise BSplineFitError(f"Missing expected keys in {prefix} fit results")

        # Load data into config
        config.cx_coefs_values = cx_data[config.CX_COEFS_DUMP_KEY]
        config.cz_coefs_values = cz_data[config.CZ_COEFS_DUMP_KEY]
        config.cx_knots_values = cx_data[config.CX_KNOTS_DUMP_KEY]
        config.cz_knots_values = cz_data[config.CZ_KNOTS_DUMP_KEY]

    except Exception as e:
        raise BSplineFitError(f"Error reading fit results: {str(e)}")

def check_bspline_fit_exists(config: BSplineAeroWriterConfig) -> bool:
    """Check if both CX and CZ B-spline fit files exist.

    Args:
        config: Configuration object with file paths

    Returns:
        True if both files exist, False otherwise
    """
    try:
        root_path = get_root_path(config)
        sub_folder = root_path / config.SUB_FOLDER
        cx_path = sub_folder / config.CX_BSPLINE_FIT_RESULTS_PATH
        cz_path = sub_folder / config.CZ_BSPLINE_FIT_RESULTS_PATH
        return cx_path.exists() and cz_path.exists()
    except BSplineFitError:
        return False

def update_database_for_bspline(config: BSplineAeroWriterConfig) -> None:
    """Create or update database with B-spline configuration.

    Args:
        config: Configuration object containing data to store

    Raises:
        BSplineFitError: If database operations fail
    """
    try:
        root_path = get_root_path(config)
        db_path = root_path / config.DB_FILENAME

        with sqlite3.connect(db_path) as conn:
            cursor = conn.cursor()

            # Drop and recreate table
            cursor.execute(f'DROP TABLE IF EXISTS {config.BSPLINE_AERO_TABLE_NAME}')
            cursor.execute(f'''
                CREATE TABLE {config.BSPLINE_AERO_TABLE_NAME} (
                    id INTEGER PRIMARY KEY,
                    {config.CX_COEFS_DUMP_KEY} TEXT,
                    {config.CX_KNOTS_DUMP_KEY} TEXT,
                    {config.CZ_COEFS_DUMP_KEY} TEXT,
                    {config.CZ_KNOTS_DUMP_KEY} TEXT,
                    {config.SCALE_FACTOR_DUMP_KEY} REAL
                )
            ''')

            # Insert data
            cursor.execute(f'''
                INSERT INTO {config.BSPLINE_AERO_TABLE_NAME}
                (id, {config.CX_COEFS_DUMP_KEY}, {config.CX_KNOTS_DUMP_KEY},
                {config.CZ_COEFS_DUMP_KEY}, {config.CZ_KNOTS_DUMP_KEY},
                {config.SCALE_FACTOR_DUMP_KEY})
                VALUES (1, ?, ?, ?, ?, ?)
            ''', (
                to_csv(config.cx_coefs_values),
                to_csv(config.cx_knots_values),
                to_csv(config.cz_coefs_values),
                to_csv(config.cz_knots_values),
                config.SCALE_FACTOR_VALUE
            ))

        print(f"[B-spline Aero Writer] Database updated successfully at {db_path}")

    except Exception as e:
        raise BSplineFitError(f"Database update failed: {str(e)}")

def main() -> None:
    """Main function to run the B-spline configuration update process."""
    try:
        config = BSplineAeroWriterConfig()
        read_bspline_fit_results(config)
        update_database_for_bspline(config)
    except BSplineFitError as e:
        print(f"Error: {str(e)}")
        exit(1)

if __name__ == "__main__":
    main()

    # Read and display the bspline fit results
