#!/usr/bin/env bash
set -euo pipefail
case "${AP_PNC_DIR:?Set absolute AP_PNC_DIR}" in
    /*) ;;
    *) echo 'AP_PNC_DIR must be absolute' >&2; exit 2 ;;
esac
src="$AP_PNC_DIR/.artifacts/px4_src"
export HOME="$AP_PNC_DIR/.artifacts/px4_home"
export XDG_RUNTIME_DIR="$HOME/runtime"
install -d -o px4 -g px4 -m 700 "$HOME" "$XDG_RUNTIME_DIR"
export GZ_SIM_RESOURCE_PATH="$src/Tools/simulation/gz/models:$src/Tools/simulation/gz/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
if [[ -f "$src/build/px4_sitl_default/gz_env.sh" ]]; then
    # Generated upstream shell scripts are not guaranteed to support nounset.
    set +u; source "$src/build/px4_sitl_default/gz_env.sh"; set -u
fi
case "${1:-server}" in
    server)
        install -d -o px4 -g px4 "$src/build/px4_sitl_default/rootfs/log"
        cd "$src"
        exec gosu px4 make px4_sitl "${PX4_GZ_TARGET:-gz_x500}"
        ;;
    gui) exec gosu px4 gz sim -g ;;
    heartbeat) exec gosu px4 python3 "$AP_PNC_DIR/infra/sim_infra/gazebo/scripts/gcs_heartbeat.py" ;;
    *) exec gosu px4 "$@" ;;
esac
