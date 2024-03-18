if [ $ROS_VERSION == 1 ] || [ $ROS_DISTRO == 'foxy' ]; then
  echo "ROS_DISTRO is $ROS_DISTRO! DON'T SETUP ACADOS"
  export TAILSITTER_PLANNING_DIR="/home/$USER/pnc_tailsitter"
  export NMPC_DIR="$TAILSITTER_PLANNING_DIR/src/nmpc"
  export PATH=$PATH:$NMPC_DIR/scripts/
  export PYTHONPATH="$PYTHONPATH:$NMPC_DIR"

## ROS2 setting, that is can run code without docker environment
elif [ -n "$USER" ]; then
  export ACADOS_SOURCE_DIR="/home/$USER/acados"
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$ACADOS_SOURCE_DIR/lib"

  source /home/$USER/env/bin/activate


  export TAILSITTER_PLANNING_DIR="/home/$USER/pnc_tailsitter"
  export NMPC_DIR="$TAILSITTER_PLANNING_DIR/src/nmpc"
  export PYTHONPATH="$PYTHONPATH:$NMPC_DIR"

  source $TAILSITTER_PLANNING_DIR/install/setup.bash
  export PX4_ROOT="/home/$USER/px4-v1.14.0-stable"

  alias nmpc_root='cd $NMPC_DIR'
  alias tailstart='python $TAILSITTER_PLANNING_DIR/src/nmpc/autostart.py'
else
  ## when USER is null, the script is run by root, (specifically docker root)
  echo "WARNING! You are running in the docker root mode, please pass the name of host name of the container."
  if [ $# -eq 1 ]; then
      HOST=$1
      export ACADOS_SOURCE_DIR="/root/acados"
      export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$ACADOS_SOURCE_DIR/lib"
      source /root/env/bin/activate
      export TAILSITTER_PLANNING_DIR="/home/$HOST/pnc_tailsitter"
      export NMPC_DIR="$TAILSITTER_PLANNING_DIR/src/nmpc"
      export PYTHONPATH="$PYTHONPATH:$NMPC_DIR"
      source $TAILSITTER_PLANNING_DIR/install/setup.bash
      alias nmpc_root='cd $NMPC_DIR'

      echo "ACADOS_SOURCE_DIR: $ACADOS_SOURCE_DIR"
      echo "TAILSITTER_PLANNING_DIR: $TAILSITTER_PLANNING_DIR"
  else
      echo "No argument! Break"
      # 添加其他针对没有参数的操作
  fi

  
fi
alias conn='python $NMPC_DIR/utils/px4_interfaces.py'
alias exp='python $NMPC_DIR/experiments/minco_exp.py'
alias print_metrics='python $NMPC_DIR/metrics/rmse.py'
export PATH=$PATH:$NMPC_DIR/scripts/
alias dds_serial='MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600'
