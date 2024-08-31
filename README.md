# Preliminary
## Clone
```shell
git clone git@github.com:SYSU-HILAB/minco-ros2.git --recursive
```



## Ensure Environment Variables are correctly set
```shell
export TAILSITTER_PLANNING_DIR=[this-project-root]
```
if you are in the root of this project.
```shell
export TAILSITTER_PLANNING_DIR=$PWD
```

## Install ros2
```shell
source $TAILSITTER_PLANNING_DIR/install-ros.sh
```
# Build & Install dependencies

## first install Miniconda
```shell
bash Miniconda3-latest-Linux-x86_64.sh
``` 
**WARNING**: not automatically activate conda

## install autodiff
```shell
source $HOME/miniconda3/bin/activate
conda install conda-forge::autodiff
source $HOME/miniconda3/bin/deactivate
``` 

## Link default colcon config
```shell
mkdir $HOME/.colcon
ln -s $TAILSITTER_PLANNING_DIR/colcon_default.yaml $HOME/.colcon/default.yaml
``` 

## install Acados & PX4 & others
```shell
source $TAILSITTER_PLANNING_DIR/install-others.sh
```

# BUILD
## All
```shell
colcon build 
```


## package specifics
```shell
colcon build --packages-select [pkg]
```
for example:
```shell
colcon build --packages-select tailsitter_df
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
srunall
```

```shell
srunall
```

## Problem
update universe config
- nmpc/model.py
- px4_interface traj_buffer