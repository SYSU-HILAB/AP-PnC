#!/usr/bin/bash

if [ -d "$ACADOS_SOURCE_DIR/build" ]; then
  echo "Acados Framework build folder exists, not start to build"
else
  echo "Acados Framework build folder not exists, start to build"
  mkdir -p $ACADOS_SOURCE_DIR/build
  cd $ACADOS_SOURCE_DIR/build
  cmake -DACADOS_WITH_QPOASES=ON ..
  make install -j4
fi

if [ ! -d "$TAILSITTER_PLANNING_DIR/.acados_venv" ]; then
  python3 -m venv $TAILSITTER_PLANNING_DIR/.acados_venv
else
  echo "Virtual environment for acados exists"
fi


source $TAILSITTER_PLANNING_DIR/.acados_venv/bin/activate
if ! pip list | grep acados > /dev/null 2>&1; then
  pip install -e $ACADOS_SOURCE_DIR/interfaces/acados_template
  pip install numpy pandas
else 
  echo "Acados python interface is already installed"
fi
deactivate


