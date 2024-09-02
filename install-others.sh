#!/usr/bin/bash

export TAILSITTER_PLANNING_DIR="$HOME/minco-ros2"
export ACADOS_SOURCE_DIR="$TAILSITTER_PLANNING_DIR/mpc_framework"

source $HOME/miniconda3/bin/activate
conda install -c conda-forge autodiff
conda deactivate


# if build folder exists, not start to build
if [ -d "$ACADOS_SOURCE_DIR/build" ]; then
  echo "acados build folder exists, not start to build"
else
  echo "acados build folder not exists, start to build"
  mkdir -p $ACADOS_SOURCE_DIR/build
  cd $ACADOS_SOURCE_DIR/build
  cmake -DACADOS_WITH_QPOASES=ON ..
  make install -j4
fi

cd $HOME


if [ -d "$HOME/px4-v1.14.0-stable" ]; then
  echo "PX4 is already setup"
else
  git clone --branch n --recursive git@github.com:SYSU-HILAB/px4-v1.14.0-stable.git \
  $HOME/px4-v1.14.0-stable
  echo "Start to setup PX4"
  cd px4-v1.14.0-stable
  sudo bash Tools/setup/ubuntu.sh
  ./install-extras.bash
fi

if [ ! -d "$HOME/acados_env" ]; then
  virtualenv $HOME/acados_env --python=/usr/bin/python3
else
  echo "Virtual environment for acados exists"
fi

source $HOME/acados_env/bin/activate
if ! pip list | grep acados > /dev/null 2>&1; then
  pip install -e $ACADOS_SOURCE_DIR/interfaces/acados_template
  pip install numpy
  pip install pandas
else 
  echo "Acados python interface is already installed"
fi
deactivate


if ! grep -q "source $TAILSITTER_PLANNING_DIR/pnc_setup.bash" ~/.bashrc; then
    echo "source $TAILSITTER_PLANNING_DIR/pnc_setup.bash" >> ~/.bashrc
fi
