#!/usr/bin/env bash
set -e

# Source ROS2 Humble base setup
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source workspace install overlay if present
if [ -f "/workspace/install/setup.bash" ]; then
    source /workspace/install/setup.bash
fi

exec "$@"
