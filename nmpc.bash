export CC="/usr/lib/ccache/gcc"
export CXX="/usr/lib/ccache/g++"
export CCACHE_DIR="$HOME/.cache/ccache/"

if [ $ROS_VERSION == 1 ] || [ $ROS_DISTRO == 'foxy' ]; then
  echo "ROS_DISTRO is $ROS_DISTRO! DON'T SETUP ACADOS"
  export TAILSITTER_PLANNING_DIR="/home/$USER/minco-ros2"
  export NMPC_DIR="$TAILSITTER_PLANNING_DIR/src/nmpc"
  export PATH=$PATH:$NMPC_DIR/scripts/
  export PYTHONPATH="$PYTHONPATH:$NMPC_DIR"

## ROS2 setting, that is can run code without docker environment
else
  export ACADOS_SOURCE_DIR="$HOME/acados"
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

  source $TAILSITTER_PLANNING_DIR/install/setup.bash
  export PX4_ROOT="$HOME/px4-v1.14.0-stable"

  alias nmpc_root='cd $NMPC_DIR'
  alias tailstart='python $TAILSITTER_PLANNING_DIR/src/nmpc/autostart.py'
  
fi
export LD_LIBRARY_PATH=$TAILSITTER_PLANNING_DIR/c_generated_code:$LD_LIBRARY_PATH
alias conn='python $NMPC_DIR/utils/px4_interfaces.py'
alias exp='python $NMPC_DIR/experiments/minco_exp.py'
alias print_metrics='python $NMPC_DIR/metrics/rmse.py'
export PATH=$PATH:$NMPC_DIR/scripts/:$TAILSITTER_PLANNING_DIR/scripts
alias dds_serial='MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600'
