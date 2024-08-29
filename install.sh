#!/usr/bin/bash
# sudo apt update -y && sudo apt install ccache 

export TAILSITTER_PLANNING_DIR="$HOME/minco-ros2"
export ACADOS_SOURCE_DIR="$TAILSITTER_PLANNING_DIR/mpc_framework"

echo $TAILSITTER_PLANNING_DIR
echo $ACADOS_SOURCE_DIR
cd $ACADOS_SOURCE_DIR
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON ..
# add more optional arguments e.g. -DACADOS_WITH_OSQP=OFF/ON -DACADOS_INSTALL_DIR=<path_to_acados_installation_folder> above
make install -j4
