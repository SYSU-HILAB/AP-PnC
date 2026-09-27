"""
Common utilities for feature development.

The modules in this package are intentionally lightweight to keep the
`feature/` tree self-contained and to avoid ad-hoc path munging between
feature prototypes and the production `src/` packages.
"""

from . import math_utils

try:
    from . import ros_viz_utils
except ImportError:
    ros_viz_utils = None  # ROS2 packages not available on host

from .color_print import Colors, color_print

__all__ = ["math_utils", "ros_viz_utils", "Colors", "color_print"]
