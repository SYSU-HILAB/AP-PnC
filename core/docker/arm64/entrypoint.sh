#!/usr/bin/env bash
set -e

# Source ROS2 Humble base setup
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# AP_PNC_DIR is the sole project path variable; never interpret it against cwd.
case "${AP_PNC_DIR:?Set absolute AP_PNC_DIR}" in
    /*) ;;
    *) echo "AP_PNC_DIR must be absolute" >&2; exit 1 ;;
esac

# Source the artifact-owned workspace overlay if present.
if [ -f "${AP_PNC_DIR}/.artifacts/colcon/install/setup.bash" ]; then
    source "${AP_PNC_DIR}/.artifacts/colcon/install/setup.bash"
fi

exec "$@"
