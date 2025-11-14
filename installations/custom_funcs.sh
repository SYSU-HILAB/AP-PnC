# Add custom shell functions for convenience

# cbp - colcon build --packages-select
cbp() {
    colcon build --packages-select "$@"
}

# pl - ros2 run plotjuggler plotjuggler
pl() {
    ros2 run plotjuggler plotjuggler "$@"
}

# restart_ros - ros2 daemon stop && ros2 daemon start
restart_ros() {
    ros2 daemon stop && ros2 daemon start
}
