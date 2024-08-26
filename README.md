# Preliminary
## Clone
```shell
git clone git@github.com:SYSU-HILAB/minco-ros2.git --recursive
```

## Ensure Environment Variables are correctly set
```shell
export TAILSITTER_PLANNING_DIR=[this-project-root]
```

## Install micro-xrce-dds-agent
```shell
sudo snap remove micro-xrce-dds-agent
sudo snap install core
sudo snap refresh
sudo snap install micro-xrce-dds-agent
snap list micro-xrce-dds-agent
```
Or
```shell
 git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
 cd Micro-XRCE-DDS-Agent
  mkdir build && cd build
  cmake ..
  make
  sudo make install
  sudo ldconfig /usr/local/lib/
```

## Install spdlog
```shell
sudo apt update
sudo apt install libspdlog-dev libfmt-dev
```

## install Ccache
Ccache is a compiler cache that can significantly speed up recompilation by caching previous compilations and reusing them when the same compilation is being done again. It's highly recommended for developers looking to optimize their build times, unless there's a specific reason to avoid it.
(https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/others/advanced-usage-of-colcon/)[Ccache]
```shell
sudo apt update && sudo apt install ccache
```
## install miniconda
[miniconda install](https://docs.anaconda.com/free/miniconda/miniconda-install/)
Be sure to install miniconda at `$HOME/miniconda3`
```shell
bash Miniconda3-latest-Linux-x86_64.sh
```
All `yes` to enable conda `python`, it will look like this

```shell
(base) root@DESKTOP-FEB761L:~/minco-ros2 #
``` 

## install autodiff
```shell
conda install conda-forge::autodiff
``` 

## set miniconda not activate on startup  or you can conda deactivate every time
```shell
echo 'conda config --set auto_activate_base false' >> ~/.bashrc

echo 'export PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages' >> ~/.bashrc
```

# Clone the repository
```shell
git clone git@github.com:SYSU-HILAB/minco-ros2.git ~/pnc_tail-sitter
```


# BUILD ACADOS WITH OPENMP
## install acados
```shell

git clone https://github.com/acados/acados.git
cd acados
git checkout v0.3.0
git submodule update --recursive --init

```

```shell
mkdir -p build
cd build
cmake -DACADOS_WITH_OSQP=ON  -DACADOS_WITH_QPOASES=ON ..
# add more optional arguments e.g. -DACADOS_WITH_OSQP=OFF/ON -DACADOS_INSTALL_DIR=<path_to_acados_installation_folder> above
make install -j4
```

## install acados python interface
Follow link [acados python interface installation](https://docs.acados.org/python_interface/index.html)

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
Typically, when building ROS2 packages with colcon, it is beneficial to enable 'symlink-install' and use CMake arguments to export compile_commands.json. Additionally, setting the build type to 'Release' can optimize performance. To streamline this process, you can create a configuration file in your home directory. Start by making a `.colcon` directory in your home folder. Within this directory, create a file named `defaults.yaml`. For instance:
```shell
mkdir ~/.colcon && vim ~/.colcon/defaults.yaml
```
paste the following to your file.
```yaml
build:
  cmake-args:
    - "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
    # - "-DCMAKE_BUILD_TYPE=Release"
  symlink-install: true
```
# GCOPTER ROS2
gcopter is from https://github.com/ZJU-FAST-Lab/GCOPTER/tree/main
used in ROS2
```shell
colcon build --symlink-install
source install/setup.bash
```



# SIMULATION

## simulator
## gazebo simulation
First, start the gazebo simulation
```shell
runpx4 1
```
## simple

## 
Second, (wait the gui on)
# Experiments
```shell
runall
```

## Problem
update universe config
- nmpc/model.py
- px4_interface traj_buffer