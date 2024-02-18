# GCOPTER ROS2

gcopter is from https://github.com/ZJU-FAST-Lab/GCOPTER/tree/main

used in ROS2

```shell
colcon build --symlink-install
source install/setup.bash
```

or in Release build
```shell
colcon build --symlink-install --cmake-args CMAKE_BUILD_TYPE Release
source install/setup.bash
```

## Launch


```shell
ros2 launch gcopter global_planning_launch.py 
```
