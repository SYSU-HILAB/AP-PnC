import sqlite3
import os
from dataclasses import dataclass, field
from typing import List
import numpy as np

@dataclass
class BSplieAeroWriterConfig:
    # Database configuration constants
    ROOT_DIR: str = 'TAILSITTER_PLANNING_DIR'
    SUB_FOLDER: str = 'src/aerodynamics'
    DB_FILENAME: str = 'meta_params.db'
    BSPLINE_AERO_TABLE_NAME: str = 'bspline_aero_config'

    CX_BSPLINE_FIT_RESULTS_PATH: str = 'data/bspline_fit_results/bspline_fit_cx.npz'
    CZ_BSPLINE_FIT_RESULTS_PATH: str = 'data/bspline_fit_results/bspline_fit_cz.npz'

    CX_KNOTS_DUMP_KEY: str = 'cx_knots'
    CZ_KNOTS_DUMP_KEY: str = 'cz_knots'

    CX_COEFS_DUMP_KEY: str = 'cx_coefs'
    CZ_COEFS_DUMP_KEY: str = 'cz_coefs'
    
    cx_coefs_values: np.ndarray = np.array([])
    cz_coefs_values: np.ndarray = np.array([])
    cx_knots_values: np.ndarray = np.array([])
    cz_knots_values: np.ndarray = np.array([])

# Method to convert the list of values to CSV format (string)
def to_csv(data_list):
    return ','.join(map(str, data_list))

def read_bspline_fit_results(config: BSplieAeroWriterConfig):
    """
    Read and display the contents of the CX bspline fit results file.
    
    Args:
        config: BSplieAeroWriterConfig object containing the file path
    """
    # Get the environment variable
    tailsitter_planning_dir = os.getenv(config.ROOT_DIR)
    
    if not tailsitter_planning_dir:
        print(f"Error: {config.ROOT_DIR} environment variable is not set.")
        return
    sub_folder = os.path.join(tailsitter_planning_dir, config.SUB_FOLDER)       
    # Construct full path to the fit results file
    fit_results_path = os.path.join(sub_folder, config.CX_BSPLINE_FIT_RESULTS_PATH)
    
    if not os.path.exists(fit_results_path):
        print(f"Error: Fit results file not found at {fit_results_path}")
        return
        
    try:
        # Load the .npz file
        data = np.load(fit_results_path)
        # Check if the keys in data match the expected dump keys from config
        expected_cx_keys = [config.CX_KNOTS_DUMP_KEY, config.CX_COEFS_DUMP_KEY]
        actual_keys = data.files
        
        if not all(key in actual_keys for key in expected_cx_keys):
            print(f"Warning: Missing expected keys in CX fit results file.")
            print(f"Expected keys: {expected_cx_keys}")
            print(f"Found keys: {actual_keys}")
            return
            
        # Load CZ fit results to check those keys as well
        cz_fit_results_path = os.path.join(sub_folder, config.CZ_BSPLINE_FIT_RESULTS_PATH)
        if os.path.exists(cz_fit_results_path):
            cz_data = np.load(cz_fit_results_path)
            expected_cz_keys = [config.CZ_KNOTS_DUMP_KEY, config.CZ_COEFS_DUMP_KEY]
            actual_cz_keys = cz_data.files
            
            if not all(key in actual_cz_keys for key in expected_cz_keys):
                print(f"Warning: Missing expected keys in CZ fit results file.")
                print(f"Expected keys: {expected_cz_keys}")
                print(f"Found keys: {actual_cz_keys}")
                return

        config.cx_coefs_values = data[config.CX_COEFS_DUMP_KEY]
        config.cz_coefs_values = cz_data[config.CZ_COEFS_DUMP_KEY]
        config.cx_knots_values = data[config.CX_KNOTS_DUMP_KEY]
        config.cz_knots_values = cz_data[config.CZ_KNOTS_DUMP_KEY]  
        
    except Exception as e:
        print(f"Error reading fit results file: {str(e)}")

# Function to create or update the database with the configuration
def create_or_update_database(config: BSplieAeroWriterConfig):
    # Get the environment variable
    tailsitter_planning_dir = os.getenv(config.ROOT_DIR)

    if not tailsitter_planning_dir:
        print(f"Error: {config.ROOT_DIR} environment variable is not set.")
        return

    # Ensure the directory exists
    if not os.path.exists(tailsitter_planning_dir):
        print(f"Error: The directory {tailsitter_planning_dir} does not exist.")
        return

    # Path to the database file
    db_file = os.path.join(tailsitter_planning_dir, config.DB_FILENAME)

    # Connect to SQLite database (it will create the file if it doesn't exist)
    conn = sqlite3.connect(db_file)
    cursor = conn.cursor()

    # Drop the existing table if it exists
    cursor.execute(f'DROP TABLE IF EXISTS {config.BSPLINE_AERO_TABLE_NAME}')
    
    # Create the table with the correct schema
    cursor.execute(f'''
        CREATE TABLE {config.BSPLINE_AERO_TABLE_NAME} (
            id INTEGER PRIMARY KEY,
            {config.CX_COEFS_DUMP_KEY} TEXT,
            {config.CX_KNOTS_DUMP_KEY} TEXT,
            {config.CZ_COEFS_DUMP_KEY} TEXT,
            {config.CZ_KNOTS_DUMP_KEY} TEXT
        )
    ''')

    # Convert the BSplieAeroWriterConfig object to CSV strings
    cx_coefs_csv = to_csv(config.cx_coefs_values)
    cx_knots_csv = to_csv(config.cx_knots_values)
    cz_coefs_csv = to_csv(config.cz_coefs_values)
    cz_knots_csv = to_csv(config.cz_knots_values)

    # Use INSERT OR REPLACE to overwrite the existing data with the new configuration
    cursor.execute(f'''
        INSERT OR REPLACE INTO {config.BSPLINE_AERO_TABLE_NAME} (id, {config.CX_COEFS_DUMP_KEY}, {config.CX_KNOTS_DUMP_KEY}, {config.CZ_COEFS_DUMP_KEY}, {config.CZ_KNOTS_DUMP_KEY})
        VALUES (1, ?, ?, ?, ?)
    ''', (cx_coefs_csv, cx_knots_csv, cz_coefs_csv, cz_knots_csv))

    # Commit the transaction
    conn.commit()

    # Close the database connection
    conn.close()

    print(f"Database '{db_file}' updated with new configuration.")

# Main function to run the script
if __name__ == "__main__":

    config = BSplieAeroWriterConfig()
    # Create or update the database with the configuration
    read_bspline_fit_results(config)
    create_or_update_database(config)
    
    # Read and display the bspline fit results
