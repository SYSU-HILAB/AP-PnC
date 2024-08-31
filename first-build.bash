#! /usr/bin/bash

source /opt/ros/humble/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/humble/

# Disable Miniconda
export PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages
export ACADOS_SOURCE_DIR="$TAILSITTER_PLANNING_DIR/mpc_framework"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$ACADOS_SOURCE_DIR/lib"

if [ -f "$HOME/env/bin/activate" ]; then
echo "You are in a host machine, running acados in virtual python environment."
source $HOME/env/bin/activate
else
echo "You are in a container or WSL machine, running acados in intrinsic python environment."
fi

export TAILSITTER_PLANNING_DIR="$HOME/minco-ros2"
export NMPC_DIR="$TAILSITTER_PLANNING_DIR/src/nmpc"
export PYTHONPATH="$PYTHONPATH:$NMPC_DIR"
export LD_LIBRARY_PATH=$TAILSITTER_PLANNING_DIR/c_generated_code:$LD_LIBRARY_PATH
export PATH=$PATH:$NMPC_DIR/scripts/:$TAILSITTER_PLANNING_DIR/scripts
pip install -e $ACADOS_SOURCE_DIR/interfaces/acados_template/

colcon build