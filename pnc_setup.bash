# ROS2 setting 

export CC="/usr/lib/ccache/gcc"
export CXX="/usr/lib/ccache/g++"
export CCACHE_DIR="$HOME/.cache/ccache/"
source /opt/ros/humble/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/humble/

if [ $ROS_DISTRO == "humble" ]; then
  echo "ROS2 version is humble"
else
  echo "ROS2 version is not humble, please install humble version"
fi

## ROS2 setting, that is can run code without docker environment
export TAILSITTER_PLANNING_DIR="$HOME/minco-ros2"
export ACADOS_SOURCE_DIR="$TAILSITTER_PLANNING_DIR/mpc_framework"
export PX4_ROOT="$HOME/px4-v1.14.0-stable"
export NMPC_DIR="$TAILSITTER_PLANNING_DIR/src/nmpc"
export PYTHONPATH="$PYTHONPATH:$NMPC_DIR"
export LD_LIBRARY_PATH=$TAILSITTER_PLANNING_DIR/c_generated_code:$LD_LIBRARY_PATH:"$ACADOS_SOURCE_DIR/lib"

if [ -f $TAILSITTER_PLANNING_DIR/install/setup.bash ]; then
  source $TAILSITTER_PLANNING_DIR/install/setup.bash
  echo "Successfully source the environment variables of the project."
else
  echo "Please build & install the project"
fi

export PATH=$PATH:$TAILSITTER_PLANNING_DIR/scripts/tools:$PX4_ROOT/Tools/

alias cbp='colcon build --packages-select'
alias pl='ros2 run plotjuggler plotjuggler'
alias restart_ros='ros2 daemon stop && ros2 daemon start'
