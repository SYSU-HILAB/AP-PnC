#!/usr/bin/bash
# sudo apt update -y && sudo apt install ccache 
git config --global user.email 632029496@qq.com
git config --global user.name WarriorHanamy


if [ -f "Miniconda3-latest-Linux-x86_64.sh" ]; then
  echo "Miniconda3-latest-Linux-x86_64.sh exists!"  
else
  echo "Installing Miniconda3-latest-Linux-x86_64.sh"
  curl -O https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
fi
# curl -O https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
# conda install conda-forge::autodiff

export TAILSITTER_PLANNING_DIR="$HOME/minco-ros2"
export ACADOS_SOURCE_DIR="$TAILSITTER_PLANNING_DIR/mpc_framework"


# if build folder exists, not start to build
if [ -d "$ACADOS_SOURCE_DIR/build" ]; then
  echo "build folder exists, not start to build"
else
  echo "build folder not exists, start to build"
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

