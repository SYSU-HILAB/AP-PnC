# Preliminary
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

## install yaml-cpp
```shell
cd ~
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build
cd build
cmake ..
make
sudo make install
```

## install ompl
```shell
sudo apt install ros-humble-ompl
```

# Clone the repository
Notice that generally you are root in docker container and $USER in normal case. Then the $HOME is always pointed to the right place.
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
cmake -DACADOS_WITH_OSQP=ON -DACADOS_WITH_OPENMP=ON -DACADOS_WITH_QPOASES=ON ..
# add more optional arguments e.g. -DACADOS_WITH_OSQP=OFF/ON -DACADOS_INSTALL_DIR=<path_to_acados_installation_folder> above
make install -j4
```

## install acados python interface
Follow link (https://docs.acados.org/python_interface/index.html)[acados python interface installation]



# parameter independent of ROS/ROS2
## jbeder/yaml-cpp
### INSTALL
```shell
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build
cd build
cmake ..
make
sudo make install
```

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





# START SIMULATION
First, generate the ref_trajecotry `control_traj`
```shell
ros2 launch trajectory_server planning.launch.py
```

Second, 
