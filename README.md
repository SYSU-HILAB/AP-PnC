# Installations
## QGC
Enable serial withou sudo
```bash
sudo usermod -aG dialout "$(id -un)"
```
QGC deps
```bash
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
```
Optional
```bash
sudo apt update
sudo apt install -y libxcb-xinerama0 libxcb-icccm4 libxcb-image0 libxcb-shm0 libxcb-keysyms1 libxcb-randr0 libxcb-shape0 libxcb-sync1 libxcb-util1 libxcb-xfixes0
sudo apt update
sudo apt install -y libxcb-xinerama0 libxcb-icccm4 libxcb-image0 libxcb-shm0 libxcb-keysyms1 libxcb-randr0 libxcb-shape0 libxcb-sync1 libxcb-util1 libxcb-xfixes0
```

```bash
curl -L -O https://github.com/mavlink/qgroundcontrol/releases/download/v4.4.4/QGroundControl.AppImage
chmod +x QGroundControl-x86_64.AppImage
mv QGroundControl-x86_64.AppImage QGC
```


## External Libraries
- [BSplinebasis](https://github.com/SYSU-HILAB/BSplinebasis)  *Remove test module*
- [tomlplusplus](https://github.com/skystrife/tomlplusplus)
- [yamlcpp](https://github.com/jbeder/yaml-cpp)
- [gcopter](https://github.com/ZJU-FAST-Lab/GCOPTER)  *Heavily modified*
- [ScopeProfiler] (https://github.com/parallel101/course/blob/2d30da61b442008c003f69225e6feca20a4ca7df/stlseries/stl_map/experiment/ScopeProfiler.h)
- [CSVWriter] (https://github.com/al-eax/CSVWriter/blob/master/include/CSVWriter.h)
1. **Disable the magnetometer**: Set the EKF2_MAG_TYPE parameter to "None" to completely disable the magnetometer and prevent it from influencing the EKF2's calculations.
## Preliminary
### Clone
```shell
git clone git@github.com:SYSU-HILAB/minco-ros2.git --recursive
```
### Environment Variables
Define the environment variable TAILSITTER_PLANING_DIR to point to the root directory of the cloned repository. For example:
```shell
export AP_PNC_DIR=$PWD
```

### Install ros2
```shell
source $AP_PNC_DIR/install-ros.sh
```

## Build & Install dependencies

### Installing Miniconda
```shell
bash Miniconda3-latest-Linux-x86_64.sh
```
**WARNING**: not automatically activate conda


### Linking Colcon Configuration
Create a .colcon directory in your home directory and link the default configuration file:
```shell
mkdir $HOME/.colcon
ln -s $AP_PNC_DIR/colcon_default.yaml $HOME/.colcon/default.yaml
```

### Installing Additional Dependencies
Install Acados, PX4, and other dependencies using the provided script:
```shell
source $AP_PNC_DIR/install-others.sh
```

## Building the System:
### pre-Build
Set colcon default build behavior for you $USER
For example, copy the following content to your $USER/.colcon/defaults.yaml
```yaml
build:
  # Set the build type for all packages (can be Debug, Release, RelWithDebInfo, MinSizeRel)
  build-base: build
  install-base: install
  log-base: log
  test-result-base: test_results
  cmake-args:
    - -DCMAKE_BUILD_TYPE=Release  # Set the build type to Release
    - -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  symlink-install: true            # Create symlinks to the install space instead of copying files

# Specify parallelism (adjust based on your hardware capabilities)
parallel:
  jobs: 4  # Number of parallel jobs during the build (e.g., -j4)

test:
  # Specify where to put test results
  result-base: test_results

```

### Building Acados and MPC
When building for the first time or making changes to the nmpc (Python) code, you need to build the Acados code. Run the following script:
```shell
cocp
```

### Building All Packages
Build all packages in the workspace:
```shell
colcon build
```


### Building Specific Packages
Build a specific package:
```shell
colcon build --packages-select [pkg]
```
For example, to build the tailsitter_df package:
```shell
colcon build --packages-select tailsitter_df
```




##  Running the Simulation:

### gazebo simulation
Starting Gazebo: Start the Gazebo simulation:
```shell
runpx4 1
```
Running other codes: Once the Gazebo GUI is visible, run the experiments
### Experiments
```shell
srunall
```

### Triggering Trajectory Mode
To enable trajectory mode, run the following command
```shell
trigger
```

## Knowledge
###  Modern CMake
- [CMake Tutorial](https://kubasejdak.com/modern-cmake-is-like-inheritance#modern-cmake-targets-properties)
- [ament_cmake](https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html)
- [ament_cmake_auto](https://zhuanlan.zhihu.com/p/438191834)



# Ai-generated

## Control Theory × Software Architecture Design Mapping

| Control Theory Concept  | Software Engineering Analogy            | Definition / Objective                                               | Programming Examples / Strategies                                 |
| ----------------------- | --------------------------------------- | -------------------------------------------------------------------- | ----------------------------------------------------------------- |
| **Robust Control**      | **Robustness**                          | System can resist disturbances / uncertain inputs                    | Input validation, bounds checking, exception handling             |
| **Adaptive Control**    | **Adaptability**                        | System adapts dynamically to changes in environment or input         | Dynamic resizing, lazy initialization, modular plug-ins           |
| **Feedforward Control** | **Offline Preprocessing**               | Anticipate disturbances and pre-compensate                           | Pre-building structures, topological sort, reverse query handling |
| **Feedback Control**    | **Recursive state update**              | Use output/state to influence next input                             | DP with state feedback, rolling arrays, event-driven loops        |
| **PID Controller**      | **Classic engineering templates**       | General-purpose controllers balancing speed, accuracy, and stability | DSU (Union-Find), LRU caches, classic memory-based templates      |
| **Stability**           | **Correctness + Performance stability** | Ensure bounded, converging behavior over time                        | Avoid infinite loops, memory leaks, or non-terminating processes  |
| **Observability**       | **Testability / Debuggability**         | Ability to infer internal state from outputs                         | Logging, assertions, metrics exposure, test hooks                 |
| **Controllability**     | **Maintainability / Configurability**   | Ability to control system via external inputs                        | Config files, dependency injection, command-line interfaces       |
