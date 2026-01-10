# PNC Tailsitter Justfile
# build top algorithms image

# === ENVIRONMENT SETUP ===

# Export AP_PNC_DIR for all recipes (allows override via environment variable)
AP_PNC_DIR := env_var_or_default("AP_PNC_DIR", "")

# Find python path - dynamically finds python through .git
# Outputs the path to python interpreter
# Usage: In recipes, use $(just _python) to get the path
_python:
    #!/usr/bin/env bash
    set -euo pipefail

    # Allow override via environment variable
    if [ -n "${AP_PNC_DIR:-}" ]; then
        GIT_ROOT="$AP_PNC_DIR"
    else
        # Find original git repository root by parsing .git file
        # This works in both: original repo AND git worktrees
        git_file="$(git rev-parse --git-dir 2>/dev/null || true)"

        if [ -z "$git_file" ]; then
            # Not in git repo - use script directory
            GIT_ROOT="$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")"
        else
            # Check if we're in a worktree (git_file contains /worktrees/)
            if [[ "$git_file" =~ /worktrees/ ]]; then
                # Read commondir to find original repo's .git location
                # commondir file is in the worktree directory itself
                commondir_file="$git_file/commondir"

                if [ -f "$commondir_file" ]; then
                    commondir="$(cat "$commondir_file")"
                    # commondir is relative to the worktree git directory
                    # e.g., git_file=/home/rec/AP-PnC/.git/worktrees/shebang
                    #      commondir=../..
                    #      result=/home/rec/AP-PnC/.git (the original .git dir)
                    original_git_dir="$(cd "$git_file/$commondir" && pwd)"
                    # The repo root is the parent of the .git directory
                    GIT_ROOT="$(dirname "$original_git_dir")"
                else
                    # Fallback: parse from .git file content
                    # The .git file in worktree contains: gitdir: <path>
                    if [ -f ".git" ]; then
                        gitdir_content=$(cat ".git" | sed 's/gitdir: //')
                        # gitdir_content is something like: /home/rec/AP-PnC/.git/worktrees/shebang
                        # Go up 4 levels to get to original repo
                        GIT_ROOT="$(cd "$(dirname "$gitdir_content")/../../.." && pwd)"
                    else
                        # Last resort: use toplevel (returns worktree root in older git)
                        GIT_ROOT="$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")")"
                    fi
                fi
            else
                # Normal repo - .git is a directory at root
                GIT_ROOT="$(dirname "$git_file")"
            fi
        fi
    fi

    # Try common venv names in order
    for venv_name in ".acados_venv" ".venv" "venv" "env"; do
        PYTHON_BIN="$GIT_ROOT/$venv_name/bin/python"
        if [ -f "$PYTHON_BIN" ]; then
            echo "$PYTHON_BIN"
            exit 0
        fi
    done

    # Not found - error message
    echo "❌ Python interpreter not found!" >&2
    echo "" >&2
    echo "Git Root detected: $GIT_ROOT" >&2
    echo "Searched for venv at: $GIT_ROOT/{.acados_venv,.venv,venv,env}/bin/python" >&2
    echo "" >&2
    echo "To fix this, run one of the following:" >&2
    echo "  just install-venv    # Create virtual environment" >&2
    echo "  just install-all     # Full installation with acados" >&2
    echo "" >&2
    echo "Or set AP_PNC_DIR override:" >&2
    echo "  AP_PNC_DIR=/custom/path just <recipe>" >&2
    exit 1

# Internal validation recipe (run automatically before dependent recipes)
_check_python:
    #!/usr/bin/env bash
    # Validate python is available (will fail-fast if not)
    PYTHON_BIN=$(just _python)
    "$PYTHON_BIN" --version > /dev/null

# === SETUP RECIPES ===

install-venv:
    #!/usr/bin/env bash
    set -euo pipefail

    # Find git root (reuse logic from _python)
    git_root=$(git rev-parse --show-toplevel 2>/dev/null || true)
    if [ -z "$git_root" ]; then
        git_root="$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")"
    fi

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

    # Find git root
    git_root=$(git rev-parse --show-toplevel 2>/dev/null || true)
    if [ -z "$git_root" ]; then
        git_root="$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")"
    fi

    export AP_PNC_DIR="$git_root"
    export ACADOS_SOURCE_DIR="$AP_PNC_DIR/mpc_framework"
    bash "$git_root/installations/install-acados.sh"
    echo "✓ Installation complete"

status:
    #!/usr/bin/env bash
    echo "=== AP-PnC Environment Status ==="

    # Find original git root (same logic as _python)
    git_file="$(git rev-parse --git-dir 2>/dev/null || true)"

    if [[ "$git_file" =~ /worktrees/ ]]; then
        # In a worktree - read commondir to find original repo
        commondir_file="$git_file/commondir"
        if [ -f "$commondir_file" ]; then
            commondir="$(cat "$commondir_file")"
            original_git_dir="$(cd "$git_file/$commondir" && pwd)"
            git_root="$(dirname "$original_git_dir")"
        else
            # Fallback - this should not happen with proper worktree
            git_root="$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")"
        fi
    elif [ -z "$git_file" ]; then
        git_root="$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")"
    else
        # Normal repo
        git_root="$(dirname "$git_file")"
    fi

    current_dir="$(pwd)"
    echo "Git Root: $git_root"
    echo "Current Dir: $current_dir"

    # Check if we're in a worktree
    if [[ "$git_file" =~ /worktrees/ ]]; then
        worktree_name="$(basename "$git_file")"
        echo "Worktree: $worktree_name (branch: $(git branch --show-current))"
    else
        echo "Worktree: (main repo)"
    fi
    echo ""

    # Try to find python
    PYTHON_BIN=$(just _python 2>/dev/null || true)
    if [ -n "$PYTHON_BIN" ] && [ -f "$PYTHON_BIN" ]; then
        echo "Python: ✓ $PYTHON_BIN"
        "$PYTHON_BIN" --version
    else
        echo "Python: ✗ Not found (run 'just install-venv')"
    fi

# === DATA GENERATION COMMANDS ===

# Generate CL, CD, CM coefficients data under coordinated flight assumption (beta = 0)
generate-coordinated-data: _check_python
    #!/usr/bin/env bash
    $(just _python) -c "from feature.lu_mpc.model.sim.gd_model_jax import generate_cl_cd_cm_coordinated_data; generate_cl_cd_cm_coordinated_data()"

# Generate CL, CD, CM coefficients data using CasADi symbolic under coordinated flight assumption (beta = 0)
generate-coordinated-data-casadi: _check_python
    #!/usr/bin/env bash
    $(just _python) -c "from feature.lu_mpc.model.sim.gd_model_casadi import generate_cl_cd_cm_coordinated_data; generate_cl_cd_cm_coordinated_data()"



# Generate 3D mesh data using JAX (usage: just generate-mesh-data-jax [mesh_size])
generate-mesh-data-jax mesh_size="361": _check_python
    #!/usr/bin/env bash
    $(just _python) -c "from feature.lu_mpc.model.sim.gd_model_jax import generate_mesh_data_jax; generate_mesh_data_jax(mesh_size={{mesh_size}})"

# Generate 3D mesh data using CasADi symbolic
generate-mesh-data-casadi: _check_python
    #!/usr/bin/env bash
    $(just _python) -c "from feature.lu_mpc.model.sim.gd_model_casadi import generate_mesh_data; generate_mesh_data()"

# Generate 3D mesh data using PyTorch (usage: just generate-mesh-data-torch [mesh_size])
generate-mesh-data-torch mesh_size="361": _check_python
    #!/usr/bin/env bash
    $(just _python) -c "from feature.lu_mpc.model.sim.gd_model_torch import generate_mesh_data_torch; generate_mesh_data_torch(mesh_size={{mesh_size}})"


# === PLOTTING COMMANDS ===
# All plotting commands now skip data regeneration by default for faster execution.
# Use the --force or -f flag to regenerate data before plotting when needed.
# Example: just plot-aero --force or just plot-mesh-aero -f 45.0 custom_data.npy

# Plot CL, CD, CM coefficients vs alpha under coordinated flight assumption (beta = 0)
# Usage: just plot-aero [--force|-f] [data_file]
plot-aero force_flag="" data_file="": _check_python
    #!/usr/bin/env bash
    set -euo pipefail
    # Find git root for data paths
    git_root=$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")")

    # Check for force flag
    if [ "{{force_flag}}" = "--force" ] || [ "{{force_flag}}" = "-f" ]; then
        echo "🔄 Regenerating coordinated data..."
        just generate-coordinated-data
        if [ $# -eq 0 ]; then
            data_file="$git_root/feature/lu_mpc/.cache/coordinated_aero_data_jax.npy"
        fi
    else
        if [ -z "{{data_file}}" ]; then
            data_file="$git_root/feature/lu_mpc/.cache/coordinated_aero_data_jax.npy"
        else
            data_file="{{data_file}}"
        fi
    fi

    # Plot using the data file (module-based execution)
    $(just _python) -m feature.lu_mpc.plotting.aero_plots "$data_file" --no-show

# Plot all aerodynamic coefficients (CL, CD, CY, Cl, Cm, Cn) across AoA
# Usage: just plot-all-aero [data_file]
# Note: Data generation is no longer available - please provide existing data file
plot-all-aero data_file="all_aero_coeffs_data.npy": _check_python
    #!/usr/bin/env bash
    set -euo pipefail
    git_root=$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")")

    if [ "{{data_file}}" = "all_aero_coeffs_data.npy" ]; then
        data_file="$git_root/feature/lu_mpc/.cache/all_aero_coeffs_data.npy"
    else
        data_file="{{data_file}}"
    fi

    $(just _python) -c "from feature.lu_mpc.plotting.aero_plots import plot_all_aerodynamic_coeffs; plot_all_aerodynamic_coeffs('$data_file')"

# Plot 3D mesh surface of aerodynamic coefficients across AoA and beta
# Usage: just plot-mesh-aero [azimuth] [data_file]
# Note: Data generation is no longer available - please provide existing data file
plot-mesh-aero azimuth="-127.5" data_file="mesh_aero_data.npy": _check_python
    #!/usr/bin/env bash
    set -euo pipefail
    git_root=$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")")

    if [ "{{data_file}}" = "mesh_aero_data.npy" ]; then
        data_file="$git_root/feature/lu_mpc/.cache/mesh_aero_data.npy"
    else
        data_file="{{data_file}}"
    fi

    $(just _python) -c "from feature.lu_mpc.plotting.mesh_plots import plot_aerodynamic_mesh; plot_aerodynamic_mesh('$data_file', azimuth='{{azimuth}}')"

# Plot 3D mesh surface of aerodynamic coefficients using PyTorch for faster computation
# Usage: just plot-mesh-aero-torch [--force|-f] [azimuth] [data_file]
plot-mesh-aero-torch force_flag="" azimuth="-127.5" data_file="mesh_aero_data_torch.npy": _check_python
    #!/usr/bin/env bash
    set -euo pipefail
    git_root=$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")")

    # Check for force flag
    if [ "{{force_flag}}" = "--force" ] || [ "{{force_flag}}" = "-f" ]; then
        echo "🔄 Regenerating PyTorch mesh data..."
        just generate-mesh-data-torch
        if [ "{{data_file}}" = "mesh_aero_data_torch.npy" ]; then
            data_file="$git_root/feature/lu_mpc/.cache/mesh_aero_data_torch.npy"
        else
            data_file="{{data_file}}"
        fi
    else
        if [ "{{data_file}}" = "mesh_aero_data_torch.npy" ]; then
            data_file="$git_root/feature/lu_mpc/.cache/mesh_aero_data_torch.npy"
        else
            data_file="{{data_file}}"
        fi
    fi

    # Plot using the data file (module-based execution)
    $(just _python) -m feature.lu_mpc.plotting.mesh_plots "$data_file" --azimuth "{{azimuth}}" --backend torch --no-show

# Plot 3D mesh surface of aerodynamic coefficients using JAX for faster computation
# Usage: just plot-mesh-aero-jax [--force|-f] [azimuth] [data_file]
plot-mesh-aero-jax force_flag="" azimuth="-127.5" data_file="mesh_aero_data_jax.npy": _check_python
    #!/usr/bin/env bash
    set -euo pipefail
    git_root=$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")")

    # Check for force flag
    if [ "{{force_flag}}" = "--force" ] || [ "{{force_flag}}" = "-f" ]; then
        echo "🔄 Regenerating JAX mesh data..."
        just generate-mesh-data-jax
        if [ "{{data_file}}" = "mesh_aero_data_jax.npy" ]; then
            data_file="$git_root/feature/lu_mpc/.cache/mesh_aero_data_jax.npy"
        else
            data_file="{{data_file}}"
        fi
    else
        if [ "{{data_file}}" = "mesh_aero_data_jax.npy" ]; then
            data_file="$git_root/feature/lu_mpc/.cache/mesh_aero_data_jax.npy"
        else
            data_file="{{data_file}}"
        fi
    fi

    # Plot using the data file (module-based execution)
    $(just _python) -m feature.lu_mpc.plotting.mesh_plots "$data_file" --azimuth "{{azimuth}}" --backend jax --no-show

# Plot 3D mesh surface of aerodynamic coefficients using CasADi symbolic
# Usage: just plot-mesh-aero-casadi [--force|-f]
plot-mesh-aero-casadi force_flag="": _check_python
    #!/usr/bin/env bash
    set -euo pipefail
    git_root=$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")")

    # Check for force flag
    if [ "{{force_flag}}" = "--force" ] || [ "{{force_flag}}" = "-f" ]; then
        echo "🔄 Regenerating CasADi mesh data..."
        just generate-mesh-data-casadi
    fi

    # Plot using the data file (module-based execution)
    $(just _python) -m feature.lu_mpc.plotting.mesh_plots "$git_root/feature/lu_mpc/.cache/mesh_aero_data_casadi.npy" --backend casadi --no-show


# === COMBINED COMMANDS (FOR BACKWARD COMPATIBILITY) ===

# Generate data and plot CL, CD, CM coefficients
plot-aero-full:
    #!/usr/bin/env bash
    just generate-coordinated-data
    just plot-aero

# Plot all aerodynamic coefficients (data generation no longer available)
plot-all-aero-full:
    #!/usr/bin/env bash
    echo "⚠️  Data generation commands are no longer available. Please ensure data file exists."
    just plot-all-aero

# Plot 3D mesh surface (data generation no longer available)
plot-mesh-aero-full azimuth="-127.5":
    #!/usr/bin/env bash
    echo "⚠️  Data generation commands are no longer available. Please ensure data file exists."
    just plot-mesh-aero {{azimuth}}

# Generate data and plot 3D mesh surface (JAX)
plot-mesh-aero-jax-full azimuth="125.5":
    #!/usr/bin/env bash
    just generate-mesh-data-jax
    just plot-mesh-aero-jax {{azimuth}}

# Generate data and plot 3D mesh surface (CasADi)
plot-mesh-aero-casadi-full:
    #!/usr/bin/env bash
    just generate-mesh-data-casadi
    just plot-mesh-aero-casadi

# Generate data and plot 3D mesh surface (PyTorch)
plot-mesh-aero-torch-full azimuth="-127.5":
    #!/usr/bin/env bash
    just generate-mesh-data-torch
    just plot-mesh-aero-torch {{azimuth}}


# Compare performance between JAX and PyTorch implementations
compare-timing: _check_python
    #!/usr/bin/env bash
    $(just _python) feature/lu_mpc/tests/test_timing_comparison.py

# Compare performance with custom parameters (usage: just compare-timing-custom mesh_size num_runs)
compare-timing-custom mesh_size num_runs: _check_python
    #!/usr/bin/env bash
    $(just _python) feature/lu_mpc/tests/test_timing_comparison.py {{mesh_size}} {{num_runs}}

# Benchmark JAX performance only
benchmark-jax: _check_python
    #!/usr/bin/env bash
    $(just _python) -c "from feature.lu_mpc.model.sim.gd_model_jax import benchmark_jax_timing; benchmark_jax_timing()"

# Benchmark PyTorch performance only
benchmark-torch: _check_python
    #!/usr/bin/env bash
    $(just _python) -c "from feature.lu_mpc.model.sim.gd_model_torch import benchmark_torch_timing; benchmark_torch_timing()"

# Test vmap implementation for correctness and performance
test-vmap: _check_python
    #!/usr/bin/env bash
    $(just _python) feature/lu_mpc/tests/test_vmap_equivalence.py

# Test JAX trim functionality
test-jax-trim: _check_python
    #!/usr/bin/env bash
    $(just _python) feature/lu_mpc/jax_trim.py

# Test configurable drone model with structured configuration (usage: just test-sim-model)
test-sim-model: _check_python
    #!/usr/bin/env bash
    $(just _python) feature/lu_mpc/tests/test_configurable_model.py

# Test symbolic CasADi aerodynamic model
test-symbolic-casadi: _check_python
    #!/usr/bin/env bash
    $(just _python) feature/lu_mpc/model/sim/gd_model_casadi.py

# Test body-to-stability frame coefficient conversion correctness
test-frame-conversion: _check_python
    #!/usr/bin/env bash
    $(just _python) -c "from feature.lu_mpc.model.sim.gd_model_casadi import test_frame_conversion; test_frame_conversion()"

# Test GD model simulation with ACADOS integrator
test-sim-gd-model: _check_python
    #!/usr/bin/env bash
    $(just _python) -c "from feature.lu_mpc.model.sim.sim_gd_model import test_gd_simulation; test_gd_simulation()"

# Simulate and visualize GD model with aerodynamic data collection
# Usage: just sim-viz-gd [sim_time] [collect_aero]
sim-viz-gd sim_time="2.0" collect_aero="true": _check_python
    #!/usr/bin/env bash
    $(just _python) feature/lu_mpc/run_sim_viz.py {{sim_time}} {{collect_aero}}

# Run all tests for lu_mpc feature (usage: just test-all)
test-all: _check_python
    #!/usr/bin/env bash
    echo "🚀 Running all lu_mpc tests..."
    echo ""
    echo "Testing structured configuration..."
    $(just _python) feature/lu_mpc/tests/test_configurable_model.py
    echo ""
    echo "Testing vmap equivalence..."
    $(just _python) feature/lu_mpc/tests/test_vmap_equivalence.py
    echo ""
    echo "Testing ACADOS ENMPC..."
    cd feature/lu_mpc && just _python tests/test_acados_enmpc.py
    echo ""
    echo "🎉 All tests completed!"

# Run JAX trim analysis for aircraft equilibrium
jax-trim: _check_python
    #!/usr/bin/env bash
    export JAX_PLATFORM_NAME=cpu
    $(just _python) feature/lu_mpc/jax_trim.py


# Legacy command - now redirects to new pipeline
plot-coordinated-aero:
    #!/usr/bin/env bash
    just plot-aero-full

alias p := plot-aero
alias pa := plot-all-aero
alias pm := plot-mesh-aero
alias pt := plot-mesh-aero-torch
alias pj := plot-mesh-aero-jax
alias pc := plot-mesh-aero-casadi
alias pca := plot-coordinated-aero
alias ct := compare-timing
alias bj := benchmark-jax
alias bt := benchmark-torch
alias tv := test-vmap
alias jt := test-jax-trim
alias jtrim := jax-trim
alias tsm := test-sim-model
alias tsg := test-sim-gd-model
alias svg := sim-viz-gd
alias tsc := test-symbolic-casadi
alias tfc := test-frame-conversion
alias ta := test-all
