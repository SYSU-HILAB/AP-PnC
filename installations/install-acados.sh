#!/usr/bin/bash
set -e

if [ -d "$ACADOS_SOURCE_DIR/build" ]; then
  echo "Acados Framework build folder exists, not start to build"
else
  (
    echo "Acados Framework build folder not exists, start to build"
    mkdir -p $ACADOS_SOURCE_DIR/build
    cd $ACADOS_SOURCE_DIR/build
    cmake -DACADOS_WITH_QPOASES=ON ..
    make install -j4
  )
fi

if [ ! -d "$AP_PNC_DIR/.acados_venv" ]; then
  python3 -m venv $AP_PNC_DIR/.acados_venv
else
  echo "Virtual environment for acados exists"
fi

export TERA_VERSION="0.2.0"
export ARCH="amd64"
export TERA_LINK="https://github.com/acados/tera_renderer/releases/download/v${TERA_VERSION}/t_renderer-v${TERA_VERSION}-linux-{ARCH}"

curl -L $TERA_LINK -o $ACADOS_SOURCE_DIR/bin/t_renderer && chmod +x $ACADOS_SOURCE_DIR/bin/t_renderer

source $AP_PNC_DIR/.acados_venv/bin/activate
if ! pip list | grep acados > /dev/null 2>&1; then
  pip install -e $ACADOS_SOURCE_DIR/interfaces/acados_template
  pip install numpy pandas
else
  echo "Acados python interface is already installed"
fi
deactivate
