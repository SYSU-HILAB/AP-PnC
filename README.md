# Preliminary
## Clone
```shell
git clone git@github.com:SYSU-HILAB/minco-ros2.git --recursive
```
## Environment Variables
Define the environment variable TAILSITTER_PLANING_DIR to point to the root directory of the cloned repository. For example:
```shell
export TAILSITTER_PLANNING_DIR=$PWD
```

## Install ros2
```shell
source $TAILSITTER_PLANNING_DIR/install-ros.sh
```

# Build & Install dependencies

## Installing Miniconda
```shell
bash Miniconda3-latest-Linux-x86_64.sh
``` 
**WARNING**: not automatically activate conda


## Linking Colcon Configuration
Create a .colcon directory in your home directory and link the default configuration file:
```shell
mkdir $HOME/.colcon
ln -s $TAILSITTER_PLANNING_DIR/colcon_default.yaml $HOME/.colcon/default.yaml
``` 

## Installing Additional Dependencies
Install Acados, PX4, and other dependencies using the provided script:
```shell
source $TAILSITTER_PLANNING_DIR/install-others.sh
```

# Building the System:
## pre-Build
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

## Building Acados and MPC
When building for the first time or making changes to the nmpc (Python) code, you need to build the Acados code. Run the following script:
```shell
cocp
```

## Building All Packages
Build all packages in the workspace:
```shell
colcon build 
```


## Building Specific Packages
Build a specific package:
```shell
colcon build --packages-select [pkg]
```
For example, to build the tailsitter_df package:
```shell
colcon build --packages-select tailsitter_df
```




#  Running the Simulation:

## gazebo simulation
Starting Gazebo: Start the Gazebo simulation:
```shell
runpx4 1
```
Running other codes: Once the Gazebo GUI is visible, run the experiments
## Experiments
```shell
srunall
```

## Triggering Trajectory Mode
To enable trajectory mode, run the following command
```shell
trigger
```
