"""Prototype modules for AP-PnC feature development."""

# Set matplotlib backend before any matplotlib imports
# This prevents X11/tkagg issues in ROS2/headless environments
import os
os.environ['MPLBACKEND'] = 'Agg'
