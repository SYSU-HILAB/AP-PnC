# PNC Tailsitter Justfile
# Uses ./pyp for git-aware Python environment detection

# === ENVIRONMENT SETUP ===

# Export AP_PNC_DIR for all recipes (allows override via environment variable)
AP_PNC_DIR := env_var_or_default("AP_PNC_DIR", "")

# === SETUP RECIPES ===

install-venv:
    #!/usr/bin/env bash
    set -euo pipefail
    git_root="$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")")"
    VENV_PATH="$git_root/.acados_venv"

    if [ -d "$VENV_PATH" ]; then
        echo "✓ Virtual environment already exists at $VENV_PATH"
        "$VENV_PATH/bin/python" --version
    else
        echo "Creating virtual environment at $VENV_PATH..."
        python3 -m venv "$VENV_PATH"
        source "$VENV_PATH/bin/activate"
        pip install --upgrade pip
        echo "✓ Virtual environment created"
    fi

install-all: install-venv
    #!/usr/bin/env bash
    set -euo pipefail
    git_root="$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")")"
    export AP_PNC_DIR="$git_root"
    export ACADOS_SOURCE_DIR="$AP_PNC_DIR/mpc_framework"
    bash "$git_root/installations/install-acados.sh"
    echo "✓ Installation complete"

install-test-deps:
    ./pyp -m pip install --upgrade pytest 'ruff==0.5.4'

status:
    #!/usr/bin/env bash
    echo "=== AP-PnC Environment Status ==="
    git_file="$(git rev-parse --git-dir 2>/dev/null || true)"

    if [[ "$git_file" =~ /worktrees/ ]]; then
        commondir_file="$git_file/commondir"
        if [ -f "$commondir_file" ]; then
            commondir="$(cat "$commondir_file")"
            original_git_dir="$(cd "$git_file/$commondir" && pwd)"
            git_root="$(dirname "$original_git_dir")"
        else
            git_root="$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")"
        fi
    elif [ -z "$git_file" ]; then
        git_root="$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")"
    else
        git_root="$(dirname "$git_file")"
    fi

    current_dir="$(pwd)"
    echo "Git Root: $git_root"
    echo "Current Dir: $current_dir"

    if [[ "$git_file" =~ /worktrees/ ]]; then
        worktree_name="$(basename "$git_file")"
        echo "Worktree: $worktree_name (branch: $(git branch --show-current))"
    else
        echo "Worktree: (main repo)"
    fi
    echo ""

    PYTHON_BIN="./pyp"
    if [ -f "$PYTHON_BIN" ]; then
        echo "Python: ✓ (via ./pyp)"
        $PYTHON_BIN --version 2>/dev/null || echo "Python: ✗ Not found (run 'just install-venv')"
    else
        echo "Python: ✗ ./pyp not found"
    fi

# === DATA GENERATION COMMANDS ===

generate-coordinated-data:
    ./pyp -c "from feature.lu_mpc.model.sim.gd_model_jax import generate_cl_cd_cm_coordinated_data; generate_cl_cd_cm_coordinated_data()"

generate-coordinated-data-casadi:
    ./pyp -c "from feature.lu_mpc.model.sim.gd_model_casadi import generate_cl_cd_cm_coordinated_data; generate_cl_cd_cm_coordinated_data()"

generate-mesh-data-jax mesh_size="361":
    ./pyp -c "from feature.lu_mpc.model.sim.gd_model_jax import generate_mesh_data_jax; generate_mesh_data_jax(mesh_size={{mesh_size}})"

generate-mesh-data-casadi:
    ./pyp -c "from feature.lu_mpc.model.sim.gd_model_casadi import generate_mesh_data; generate_mesh_data()"

generate-mesh-data-torch mesh_size="361":
    ./pyp -c "from feature.lu_mpc.model.sim.gd_model_torch import generate_mesh_data_torch; generate_mesh_data_torch(mesh_size={{mesh_size}})"

# === PLOTTING COMMANDS ===
# Plotting commands rely on their modules to auto-generate cached data when needed.
# Use the generate-* recipes manually if you need to refresh cached artifacts.

plot-aero force_flag="" data_file="":
    #!/usr/bin/env bash
    set -euo pipefail
    git_root="$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")")"
    data_file="$git_root/feature/lu_mpc/.cache/coordinated_aero_data_jax.npy"

    if [ "{{force_flag}}" = "--force" ] || [ "{{force_flag}}" = "-f" ]; then
        echo "🔄 Regenerating coordinated data..."
        just generate-coordinated-data
    elif [ -n "{{data_file}}" ]; then
        data_file="{{data_file}}"
    fi

    ./pyp -m feature.lu_mpc.plotting.aero_plots "$data_file" --no-show

plot-all-aero data_file="all_aero_coeffs_data.npy":
    #!/usr/bin/env bash
    set -euo pipefail
    git_root="$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")")"

    if [ "{{data_file}}" = "all_aero_coeffs_data.npy" ]; then
        data_file="$git_root/feature/lu_mpc/.cache/all_aero_coeffs_data.npy"
    else
        data_file="{{data_file}}"
    fi

    ./pyp -c "from feature.lu_mpc.plotting.aero_plots import plot_all_aerodynamic_coeffs; plot_all_aerodynamic_coeffs('$data_file')"

plot-mesh-aero azimuth="-127.5" data_file="mesh_aero_data.npy":
    #!/usr/bin/env bash
    set -euo pipefail
    git_root="$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")")"

    if [ "{{data_file}}" = "mesh_aero_data.npy" ]; then
        data_file="$git_root/feature/lu_mpc/.cache/mesh_aero_data.npy"
    else
        data_file="{{data_file}}"
    fi

    ./pyp -c "from feature.lu_mpc.plotting.mesh_plots import plot_aerodynamic_mesh; plot_aerodynamic_mesh('$data_file', azimuth='{{azimuth}}')"

plot-mesh-aero-torch azimuth="-127.5" data_file="mesh_aero_data_torch.npy":
    #!/usr/bin/env bash
    set -euo pipefail
    git_root="$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")")"

    if [ "{{data_file}}" = "mesh_aero_data_torch.npy" ]; then
        data_file="$git_root/feature/lu_mpc/.cache/mesh_aero_data_torch.npy"
    else
        data_file="{{data_file}}"
    fi

    ./pyp -c "from feature.lu_mpc.plotting.mesh_plots import plot_aerodynamic_mesh_torch; plot_aerodynamic_mesh_torch('$data_file', azimuth='{{azimuth}}', show_plot=False)"

plot-mesh-aero-jax azimuth="-127.5" data_file="mesh_aero_data_jax.npy":
    #!/usr/bin/env bash
    set -euo pipefail
    git_root="$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")")"

    if [ "{{data_file}}" = "mesh_aero_data_jax.npy" ]; then
        data_file="$git_root/feature/lu_mpc/.cache/mesh_aero_data_jax.npy"
    else
        data_file="{{data_file}}"
    fi

    ./pyp -c "from feature.lu_mpc.plotting.mesh_plots import plot_aerodynamic_mesh_jax; plot_aerodynamic_mesh_jax('$data_file', azimuth='{{azimuth}}', show_plot=False)"

plot-mesh-aero-casadi azimuth="-127.5" data_file="mesh_aero_data_casadi.npy":
    #!/usr/bin/env bash
    set -euo pipefail
    git_root="$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")")"

    if [ "{{data_file}}" = "mesh_aero_data_casadi.npy" ]; then
        data_file="$git_root/feature/lu_mpc/.cache/mesh_aero_data_casadi.npy"
    else
        data_file="{{data_file}}"
    fi

    ./pyp -c "from feature.lu_mpc.plotting.mesh_plots import plot_aerodynamic_mesh_casadi; plot_aerodynamic_mesh_casadi('$data_file', azimuth='{{azimuth}}', show_plot=False)"

compare-timing:
    ./pyp scripts/compare_timing.py

compare-timing-custom mesh_size num_runs:
    ./pyp scripts/compare_timing.py {{mesh_size}} {{num_runs}}

benchmark-jax:
    ./pyp -c "from feature.lu_mpc.model.sim.gd_model_jax import benchmark_jax_timing; benchmark_jax_timing()"

benchmark-torch:
    ./pyp -c "from feature.lu_mpc.model.sim.gd_model_torch import benchmark_torch_timing; benchmark_torch_timing()"

test-vmap:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./pyp -m pytest test/lu_mpc/model/sim/test_gd_model_jax.py

test-jax-trim:
    ./pyp test/lu_mpc/analysis/user_test_jax_trim.py

test-sim-model:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./pyp -m pytest test/lu_mpc/model/sim/test_math_utils.py

test-symbolic-casadi:
    ./pyp feature/lu_mpc/model/sim/gd_model_casadi.py

test-frame-conversion:
    ./pyp -c "from feature.lu_mpc.model.sim.gd_model_casadi import test_frame_conversion; test_frame_conversion()"

test-sim-gd-model:
    ./pyp -c "from feature.lu_mpc.model.sim.sim_gd_model import test_gd_simulation; test_gd_simulation()"

test-proposed-nmpc:
    #!/usr/bin/env bash
    set -euo pipefail
    export LD_LIBRARY_PATH=/home/rec/AP-PnC/mpc_framework/lib:$LD_LIBRARY_PATH
    ./pyp -m feature.lu_mpc.model.ctrl.test_proposed_nmpc

test-proposed-nmpc-simple:
    #!/usr/bin/env bash
    set -euo pipefail
    export LD_LIBRARY_PATH=/home/rec/AP-PnC/mpc_framework/lib:$LD_LIBRARY_PATH
    ./pyp feature/lu_mpc/model/ctrl/test_proposed_nmpc_simple.py

# test-proposed-nmpc-viz:
#     # NOTE: This recipe is disabled - use 'just viz-traj-tracking' instead
#     # The module feature.lu_mpc.model.ctrl.test_proposed_nmpc_viz does not exist
#     #!/usr/bin/env bash
#     set -euo pipefail
#     git_root="$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")")"
#     export LD_LIBRARY_PATH=/home/rec/AP-PnC/mpc_framework/lib:$LD_LIBRARY_PATH
#     export PYTHONPATH="$git_root${PYTHONPATH:+:$PYTHONPATH}"
#     echo "Starting nMPC ROS2 visualization..."
#     echo "Make sure ROS2 is sourced: source /opt/ros/humble/setup.bash"
#     ./pyp -m feature.lu_mpc.model.ctrl.test_proposed_nmpc_viz

test-ref-traj-viz:
    #!/usr/bin/env bash
    set -euo pipefail
    git_root="$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")")"
    export PYTHONPATH="$git_root${PYTHONPATH:+:$PYTHONPATH}"
    echo "Starting Reference Trajectory ROS2 visualization..."
    echo "Publishes reference trajectory states in real-time (no controller)"
    ./pyp -m feature.lu_mpc.model.ctrl.test_ref_trajectory_viz

test-ros-viz-node:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./pyp -m pytest test/lu_mpc/ros

viz-ref-traj playback_dt="0.05" config="feature/lu_mpc/ros/config/ref_trajs.rviz":
    #!/usr/bin/env bash
    set -euo pipefail
    git_root="$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")")"
    echo "Launching reference trajectory publisher + RViz2"
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-/home/rec/AP-PnC/mpc_framework}"
    export LD_LIBRARY_PATH="/home/rec/AP-PnC/mpc_framework/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export PYTHONPATH="$git_root${PYTHONPATH:+:$PYTHONPATH}"
    ./pyp test/lu_mpc/ros/user_test_ref_traj_viz.py --playback-dt {{playback_dt}} --rviz-config {{config}}

viz-traj-tracking config="feature/lu_mpc/ros/config/nmpc_viz.rviz":
    #!/usr/bin/env bash
    set -euo pipefail
    git_root="$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")")"
    echo "Launching tracking visualization + RViz2"
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-/home/rec/AP-PnC/mpc_framework}"
    export LD_LIBRARY_PATH="/home/rec/AP-PnC/mpc_framework/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export PYTHONPATH="$git_root${PYTHONPATH:+:$PYTHONPATH}"
    ./pyp test/lu_mpc/ros/user_test_traj_tracking_viz.py --rviz-config {{config}}

show_odom config="feature/lu_mpc/ros/config/plotjuggler_odometry_xyz.xml":
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Launching PlotJuggler (odometry XYZ)..."
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    ros2 run plotjuggler plotjuggler -l {{config}}

test-all:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./pyp -m pytest test/lu_mpc

lint-tests:
    ./pyp -m ruff check test

format-tests:
    ./pyp -m ruff format test

jax-trim:
    #!/usr/bin/env bash
    export JAX_PLATFORM_NAME=cpu
    ./pyp test/lu_mpc/analysis/user_test_jax_trim.py

# === ROS2/C++ BUILD AND TEST ===

build-aerodynamics:
    #!/usr/bin/env bash
    set -euo pipefail
    ./scripts/tools/build --packages-select aerodynamics --allow-overriding aerodynamics

build-trajectory-server:
    #!/usr/bin/env bash
    set -euo pipefail
    ./scripts/tools/build --packages-select trajectory_server --allow-overriding trajectory_server

build-cpp: build-aerodynamics build-trajectory-server

test-aerodynamics package="aerodynamics":
    #!/usr/bin/env bash
    set -euo pipefail
    ./scripts/tools/test {{package}}

test-cpp: test-aerodynamics

# === C++ DATA GENERATION ===

generate-aero-table:
    #!/usr/bin/env bash
    set -euo pipefail
    git_root="$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")")"
    mkdir -p "$git_root/artifacts/simulations"
    "$git_root/build/aerodynamics/generate_aero_lookup_table" "$git_root/artifacts/simulations/aero_lookup_table.csv" 10.0 2.0

# Aliases
alias p := plot-aero
alias pa := plot-all-aero
alias pm := plot-mesh-aero
alias pt := plot-mesh-aero-torch
alias pj := plot-mesh-aero-jax
alias pc := plot-mesh-aero-casadi
alias ct := compare-timing
alias bj := benchmark-jax
alias bt := benchmark-torch
alias tv := test-vmap
alias jt := test-jax-trim
alias jtrim := jax-trim
alias tsm := test-sim-model
alias tsg := test-sim-gd-model
alias tpn := test-proposed-nmpc
alias tpns := test-proposed-nmpc-simple
# alias tpnv := test-proposed-nmpc-viz  # Disabled: use 'just viz-traj-tracking' instead
# alias svg := sim-viz-gd  # Removed: target no longer exists
alias tsc := test-symbolic-casadi
alias tfc := test-frame-conversion
alias ta := test-all

# === DOCKER COMPOSE ===

docker-build:
    docker compose -f docker/docker-compose.yml build

docker-up:
    docker compose -f docker/docker-compose.yml up -d

docker-down:
    docker compose -f docker/docker-compose.yml down

# Aliases
alias db := docker-build
alias dup := docker-up
alias down := docker-down
