import sqlite3
from pathlib import Path
from dataclasses import dataclass
from typing import Tuple
import os
import traceback
import numpy as np
from numpy.typing import NDArray


@dataclass
class PhiAeroWriterConfig:
    """Configuration for phi aerodynamics data writing.
    
    Attributes:
        ROOT_DIR: Environment variable name for root directory
        SUB_FOLDER: Subfolder path for aerodynamics data
        DB_FILENAME: SQLite database filename
        PHI_AERO_TABLE_NAME: Table name for phi data
        PHI_COEFS_DUMP_KEY: Key for phi coefficients in database
    """
    ROOT_DIR: str = 'TAILSITTER_PLANNING_DIR'
    SUB_FOLDER: str = 'src/aerodynamics'
    DB_FILENAME: str = 'meta_params.db'
    PHI_AERO_TABLE_NAME: str = 'phi_aero_config'
    PHI_COEFS_DUMP_KEY: str = 'phi_coefs'
    
    phi_coefs_values: NDArray = np.array([])


class PhiAeroError(Exception):
    """Custom exception for phi aerodynamics operations."""
    def __init__(self, message: str, details: str = None):
        self.message = message
        self.details = details
        super().__init__(self.full_message)
    
    @property
    def full_message(self) -> str:
        error_msg = [f"Phi Aero Error: {self.message}"]
        if self.details:
            error_msg.append(f"Details: {self.details}")
        return "\n".join(error_msg)


def to_csv(data: NDArray) -> str:
    """Convert numpy array to CSV string format.
    
    Args:
        data: Numpy array to convert
        
    Returns:
        Comma-separated string of values
    """
    try:
        return ','.join(map(str, data.flatten('C')))
    except Exception as e:
        raise PhiAeroError("Failed to convert array to CSV", 
                          f"Error: {str(e)}\nArray shape: {data.shape}")


def get_root_path(config: PhiAeroWriterConfig) -> Path:
    """Get the root path from environment variable.
    
    Args:
        config: Configuration object containing ROOT_DIR
        
    Returns:
        Path object for root directory
        
    Raises:
        PhiAeroError: If environment variable is not set
    """
    try:
        root_dir = os.getenv(config.ROOT_DIR)
        if not root_dir:
            raise PhiAeroError(
                f"Environment variable {config.ROOT_DIR} is not set",
                "Please set the environment variable before running the script"
            )
        
        root_path = Path(root_dir)
        if not root_path.exists():
            raise PhiAeroError(
                f"Directory {root_path} does not exist",
                "Please check if the path exists and you have correct permissions"
            )
            
        return root_path
        
    except Exception as e:
        if isinstance(e, PhiAeroError):
            raise
        raise PhiAeroError("Failed to get root path", str(e))


def validate_phi_coefficients(phi_coefs: NDArray) -> Tuple[int, int]:
    """Validate phi coefficients array.
    
    Args:
        phi_coefs: Array of phi coefficients
        
    Returns:
        Tuple of (rows, cols) dimensions
        
    Raises:
        PhiAeroError: If coefficients are invalid
    """
    try:
        if phi_coefs.size == 0:
            raise PhiAeroError(
                "Phi coefficients array is empty",
                "Please ensure coefficients are properly initialized"
            )
            
        if len(phi_coefs.shape) != 2:
            raise PhiAeroError(
                "Phi coefficients must be a 2D array",
                f"Current array shape: {phi_coefs.shape}"
            )
            
        return phi_coefs.shape
        
    except Exception as e:
        if isinstance(e, PhiAeroError):
            raise
        raise PhiAeroError("Failed to validate phi coefficients", str(e))


def update_database_for_phi(config: PhiAeroWriterConfig) -> None:
    """Create or update database with phi configuration.
    
    Args:
        config: Configuration object containing data to store
        
    Raises:
        PhiAeroError: If database operations fail
    """
    try:
        root_path = get_root_path(config)
        db_path = root_path / config.DB_FILENAME
        
        # Validate coefficients before database operations
        rows, cols = validate_phi_coefficients(config.phi_coefs_values)
        phi_coefs_csv = to_csv(config.phi_coefs_values)

        with sqlite3.connect(db_path) as conn:
            cursor = conn.cursor()
            
            # Drop and recreate table
            cursor.execute(f'DROP TABLE IF EXISTS {config.PHI_AERO_TABLE_NAME}')
            cursor.execute(f'''
                CREATE TABLE {config.PHI_AERO_TABLE_NAME} (
                    id INTEGER PRIMARY KEY,
                    {config.PHI_COEFS_DUMP_KEY} TEXT,
                    rows INTEGER,
                    cols INTEGER
                )
            ''')
            
            # Insert data
            cursor.execute(f'''
                INSERT INTO {config.PHI_AERO_TABLE_NAME} 
                ({config.PHI_COEFS_DUMP_KEY}, rows, cols)
                VALUES (?, ?, ?)
            ''', (phi_coefs_csv, rows, cols))
            
        print(f"[Phi Aero Writer] Database updated successfully at {db_path}")
        
    except sqlite3.Error as e:
        raise PhiAeroError(
            "Database operation failed",
            f"SQLite error: {str(e)}\nDB path: {db_path}"
        )
    except Exception as e:
        if isinstance(e, PhiAeroError):
            raise
        raise PhiAeroError(
            "Failed to update database",
            f"Error type: {type(e).__name__}\nError: {str(e)}"
        )


# Initialize the coefficients with default values
phi_coefs_values = np.array([
    [-0.1, 0, 0],
    [0, 0, 0], 
    [0, 0, -1.0]
])


def main() -> None:
    """Main function to run the phi coefficient update process."""
    try:
        config = PhiAeroWriterConfig()
        config.phi_coefs_values = phi_coefs_values
        update_database_for_phi(config)
    except PhiAeroError as e:
        print(e.full_message)
        print("\nTraceback:")
        traceback.print_exc()
        exit(1)
    except Exception as e:
        print(f"Unexpected error: {type(e).__name__}")
        print(f"Error message: {str(e)}")
        print("\nTraceback:")
        traceback.print_exc()
        exit(1)


if __name__ == "__main__":
    main()