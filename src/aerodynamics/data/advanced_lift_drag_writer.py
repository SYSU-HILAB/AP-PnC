import sqlite3
from pathlib import Path
import os
from dataclasses import dataclass
from typing import Dict
import traceback
import numpy as np
from numpy.typing import NDArray


@dataclass
class AdvancedLiftDragWriterConfig:
    """Configuration for advanced lift drag aerodynamics data writing.

    Attributes:
        ROOT_DIR: Environment variable name for root directory
        DB_FILENAME: SQLite database filename
        TABLE_NAME: Table name for advanced lift drag data
    """
    ROOT_DIR: str = 'AP_PNC_DIR'
    DB_FILENAME: str = 'aero_sim_params.db'
    TABLE_NAME: str = 'advanced_lift_drag_config'

    # Default values from the C++ code
    params: Dict[str, float] = None

    def __post_init__(self):
        if self.params is None:
            self.params = {
                'sigmoid_blend': 15.0,
                'CL_alpha_0': 0.15188,
                'CL_alpha': 5.015,
                'alpha_stall': 15.0 * np.pi / 180.0,  # 15 degrees in radians
                'eff': np.pi * 8.5 * 0.97,
                'CD_0': 0.15,
                'CD_flat_plate': 1.28,
                'CY_beta': -0.05,
                'CL_beta_loss': 0.0,  # useless
                'scale_factor': 0.5 * 1.184 * 0.24 / 2.0 * 1.7395338636344466 * 1.6033190310032224
            }


class AdvancedLiftDragError(Exception):
    """Custom exception for advanced lift drag operations."""
    def __init__(self, message: str, details: str = None):
        self.message = message
        self.details = details
        super().__init__(self.full_message)

    @property
    def full_message(self) -> str:
        error_msg = [f"Advanced Lift Drag Error: {self.message}"]
        if self.details:
            error_msg.append(f"Details: {self.details}")
        return "\n".join(error_msg)


def get_root_path(config: AdvancedLiftDragWriterConfig) -> Path:
    """Get the root path from environment variable.

    Args:
        config: Configuration object containing ROOT_DIR

    Returns:
        Path object for root directory

    Raises:
        AdvancedLiftDragError: If environment variable is not set
    """
    try:
        root_dir = os.getenv(config.ROOT_DIR)
        if not root_dir:
            raise AdvancedLiftDragError(
                f"Environment variable {config.ROOT_DIR} is not set",
                "Please set the environment variable before running the script"
            )

        root_path = Path(root_dir)
        if not root_path.exists():
            raise AdvancedLiftDragError(
                f"Directory {root_path} does not exist",
                "Please check if the path exists and you have correct permissions"
            )

        return root_path

    except Exception as e:
        if isinstance(e, AdvancedLiftDragError):
            raise
        raise AdvancedLiftDragError("Failed to get root path", str(e))


def validate_parameters(params: Dict[str, float]) -> None:
    """Validate advanced lift drag parameters.

    Args:
        params: Dictionary of parameter values

    Raises:
        AdvancedLiftDragError: If parameters are invalid
    """
    try:
        required_params = {
            'sigmoid_blend', 'CL_alpha_0', 'CL_alpha', 'alpha_stall',
            'eff', 'CD_0', 'CD_flat_plate', 'CY_beta', 'CL_beta_loss',
            'scale_factor'
        }

        if not params:
            raise AdvancedLiftDragError(
                "Parameters dictionary is empty",
                "Please ensure parameters are properly initialized"
            )

        missing_params = required_params - set(params.keys())
        if missing_params:
            raise AdvancedLiftDragError(
                "Missing required parameters",
                f"Missing parameters: {missing_params}"
            )

        # Validate parameter types and values
        for param, value in params.items():
            if not isinstance(value, (int, float)):
                raise AdvancedLiftDragError(
                    f"Invalid parameter type for {param}",
                    f"Expected numeric type, got {type(value)}"
                )

    except Exception as e:
        if isinstance(e, AdvancedLiftDragError):
            raise
        raise AdvancedLiftDragError("Failed to validate parameters", str(e))


def update_database_for_advanced_lift_drag(config: AdvancedLiftDragWriterConfig) -> None:
    """Create or update database with advanced lift drag configuration.

    Args:
        config: Configuration object containing data to store

    Raises:
        AdvancedLiftDragError: If database operations fail
    """
    try:
        root_path = get_root_path(config)
        db_path = root_path / config.DB_FILENAME

        # Validate parameters before database operations
        validate_parameters(config.params)

        with sqlite3.connect(db_path) as conn:
            cursor = conn.cursor()

            # Drop and recreate table
            cursor.execute(f'DROP TABLE IF EXISTS {config.TABLE_NAME}')

            # Create table with columns for each parameter
            columns = ', '.join(f'{param} REAL' for param in config.params.keys())
            cursor.execute(f'''
                CREATE TABLE {config.TABLE_NAME} (
                    id INTEGER PRIMARY KEY,
                    {columns}
                )
            ''')

            # Insert data
            columns = ', '.join(config.params.keys())
            placeholders = ', '.join(['?'] * len(config.params))
            cursor.execute(f'''
                INSERT INTO {config.TABLE_NAME}
                ({columns})
                VALUES ({placeholders})
            ''', tuple(config.params.values()))

        print(f"[Advanced Lift Drag Writer] Database updated successfully at {db_path}")

    except sqlite3.Error as e:
        raise AdvancedLiftDragError(
            "Database operation failed",
            f"SQLite error: {str(e)}\nDB path: {db_path}"
        )
    except Exception as e:
        if isinstance(e, AdvancedLiftDragError):
            raise
        raise AdvancedLiftDragError(
            "Failed to update database",
            f"Error type: {type(e).__name__}\nError: {str(e)}"
        )


def main() -> None:
    """Main function to run the advanced lift drag parameter update process."""
    try:
        config = AdvancedLiftDragWriterConfig()
        update_database_for_advanced_lift_drag(config)
    except AdvancedLiftDragError as e:
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
