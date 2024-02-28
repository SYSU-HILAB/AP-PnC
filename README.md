# BUILD
## package specifics
```shell
colcon build --packages-select [pkg]
```
for example:
```shell
colcon build --packages-select tailsitter_df
```

## default.yaml to set your default colcon build 
Typically, when building ROS2 packages with colcon, it is beneficial to enable 'symlink-install' and use CMake arguments to export compile_commands.json. Additionally, setting the build type to 'Release' can optimize performance. To streamline this process, you can create a configuration file in your home directory. Start by making a .colcon directory in your home folder. Within this directory, create a file named defaults.yaml. For instance:
```yaml
build:
  cmake-args:
    - "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
  symlink-install: true
```
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
