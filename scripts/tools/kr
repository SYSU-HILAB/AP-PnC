#!/bin/bash
if [ -z "$1" ]; then
    echo "Killing all."
    KILLPX4=1
else
    echo "Killing except px4."
    KILLPX4=0
fi


if [ $KILLPX4 -eq 1 ]; then
  echo "Killing px4"
  PIDS=$(pgrep -f "px4")
  if [ -n "$PIDS" ]; then
    echo $PIDS | xargs kill
  fi

  echo "Klling gazebo"
  PIDS=$(pgrep -f "gz")
  if [ -n "$PIDS" ]; then
    echo $PIDS | xargs kill
  fi

  echo "Killing micro-xrce-dds"
  PIDS=$(pgrep -f "micro-xrce-dds")
  if [ -n "$PIDS" ]; then
    echo $PIDS | xargs kill
  fi

  PIDS=$(pgrep -f "Micro")
  if [ -n "$PIDS" ]; then
    echo $PIDS | xargs kill
  fi

  PIDS=$(pgrep -f "micro-")
  if [ -n "$PIDS" ]; then
    echo $PIDS | xargs kill
  fi
  pkill ruby
fi

echo "killing traj"
PIDS=$(pgrep -f "traj")
if [ -n "$PIDS" ]; then
  echo $PIDS | xargs kill
fi

echo "killing mocap"
PIDS=$(pgrep -f "mocap")
if [ -n "$PIDS" ]; then
  echo $PIDS | xargs kill
fi

echo "killing mpc"
PIDS=$(pgrep -f "mpc")
if [ -n "$PIDS" ]; then
  echo $PIDS | xargs kill
fi

echo "killing offboard"
PIDS=$(pgrep -f "offboard")
if [ -n "$PIDS" ]; then
  echo $PIDS | xargs kill
fi

PIDS=$(pgrep -f "ros2")
if [ -n "$PIDS" ]; then
  echo $PIDS | xargs kill
fi

echo "killing px4_interface"
PIDS=$(pgrep -f "px4_interface")
if [ -n "$PIDS" ]; then
  echo $PIDS | xargs kill
fi
