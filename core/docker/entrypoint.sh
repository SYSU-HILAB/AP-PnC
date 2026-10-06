#!/usr/bin/env bash
set -e
case "${AP_PNC_DIR:?Set absolute AP_PNC_DIR}" in
    /*) ;;
    *) echo 'AP_PNC_DIR must be absolute' >&2; exit 2 ;;
esac
export ROS_HOME="$AP_PNC_DIR/.artifacts/ros"
mkdir -p "$ROS_HOME"
source /opt/ros/humble/setup.bash
# MAVROS is a separate overlay, built before the project packages.
for overlay in \
    "$AP_PNC_DIR/.artifacts/colcon/mavros/install/setup.bash" \
    "$AP_PNC_DIR/.artifacts/colcon/install/setup.bash"; do
    if [[ -f "$overlay" ]]; then source "$overlay"; fi
done
# Sourcing this script from bashrc must not replace the interactive shell.
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then return 0; fi
exec "$@"
