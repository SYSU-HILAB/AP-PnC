# Installations
## External Libraries
All these are installed by git submodule things.
- [BSplinebasis](https://github.com/SYSU-HILAB/BSplinebasis)  *Remove test module*
- [tomlplusplus](https://github.com/skystrife/tomlplusplus)
- [yamlcpp](https://github.com/jbeder/yaml-cpp)
- [gcopter](https://github.com/ZJU-FAST-Lab/GCOPTER)  *Heavily modified*
- [ScopeProfiler] (https://github.com/parallel101/course/blob/2d30da61b442008c003f69225e6feca20a4ca7df/stlseries/stl_map/experiment/ScopeProfiler.h)
- [CSVWriter] (https://github.com/al-eax/CSVWriter/blob/master/include/CSVWriter.h)
1. **Disable the magnetometer**: Set the EKF2_MAG_TYPE parameter to "None" to completely disable the magnetometer and prevent it from influencing the EKF2's calculations.
## Preliminary
```shell
git clone https://github.com/WarriorHanamy/AP-PnC.git --recursive
```

## Build and Run Commands

The project uses Docker containers managed through Just commands. Make sure you have [Just](https://github.com/casey/just) installed.

#### Main AP-PnC Container
```bash
# Build the main AP-PnC container
just build

# Run the AP-PnC container (with GUI support)
just run
```

#### PX4 Simulator Container
```bash
# Build the PX4 simulator container
just build-px4

# Run the PX4 simulator container
just run-px4
```

#### QGroundControl Container
```bash
# Build the QGroundControl container
just build-qgc

# Run QGroundControl
just run-qgc
```

#### Manual Docker Commands
If you prefer to use Docker directly:

```bash
# Build AP-PnC container
docker build -f docker/ap-pnc.dockerfile --network=host -t ap-pnc:latest .

# Run AP-PnC container
docker run -it --rm \
    --privileged --net=host \
    -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
    -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd)/src/tailsitter_df:/ap-pnc/src/tailsitter_df \
    --name ap-pnc \
    ap-pnc bash
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
