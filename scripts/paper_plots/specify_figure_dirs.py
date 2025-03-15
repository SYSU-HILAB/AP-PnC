import os 

def root_dir():
    """Specify the directories for the figures."""
    return os.getenv('TAILSITTER_PLANNING_DIR')


def tracking_plots_dir():
    """Specify the directories for the tracking data."""
    
    root_dir = os.getenv('TAILSITTER_PLANNING_DIR')
    tracking_dir = os.path.join(root_dir, 'scripts', 'paper_plots', 'tracking_plots')
    
    if not os.path.exists(tracking_dir):
        os.makedirs(tracking_dir)
    
    return tracking_dir


    