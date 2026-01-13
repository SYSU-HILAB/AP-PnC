#!/usr/bin/env bash
#
# .env_setup.sh - Shared environment setup for AP-PnC scripts
#
# Source this file in any script that needs AP-PnC environment:
#   source "$(dirname "${BASH_SOURCE[0]}")/.env_setup.sh"
#

# Prevent multiple inclusion
[ -n "$_AP_PNC_ENV_LOADED" ] && return 0
_AP_PNC_ENV_LOADED=1

# === ANSI Color Codes ===
export AP_PNC_COLOR_RED='\033[0;91m'
export AP_PNC_COLOR_GREEN='\033[0;92m'
export AP_PNC_COLOR_YELLOW='\033[0;93m'
export AP_PNC_COLOR_BLUE='\033[0;94m'
export AP_PNC_COLOR_BOLD='\033[1m'
export AP_PNC_COLOR_RESET='\033[0m'
export AP_PNC_CHECKMARK='\u2713'

# === Git Root Detection ===

ap_pnc_find_git_root() {
    # Check for AP_PNC_DIR override
    if [ -n "${AP_PNC_DIR:-}" ]; then
        echo "$AP_PNC_DIR"
        return
    fi

    # Find git directory
    local git_file
    git_file="$(git rev-parse --git-dir 2>/dev/null || true)"

    if [ -z "$git_file" ]; then
        # Not in git - use script directory (assuming scripts/tools/ structure)
        # Get absolute path of the script and go up two levels
        local script_dir
        script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
        echo "$(dirname "$(dirname "$script_dir")")"
        return
    fi

    # Check if worktree (gitdir contains /worktrees/)
    if [[ "$git_file" =~ /worktrees/ ]]; then
        # Read commondir to find original repo
        local commondir_file="$git_file/commondir"
        if [ -f "$commondir_file" ]; then
            local commondir
            commondir="$(cat "$commondir_file")"
            local original_git_dir
            original_git_dir="$(cd "$git_file/$commondir" && pwd)"
            # Parent of .git is the repo root
            echo "$(dirname "$original_git_dir")"
            return
        fi
    fi

    # Normal repo - .git is a directory at root
    # Convert to absolute path
    (cd "$(dirname "$git_file")" && pwd)
}

# === Environment Setup ===

ap_pnc_setup_env() {
    local git_root
    git_root="$(ap_pnc_find_git_root)"

    # Export minimal required variables
    export AP_PNC_DIR="$git_root"
    export ACADOS_SOURCE_DIR="$git_root/mpc_framework"

    # Set LD_LIBRARY_PATH for runtime linking
    local lib_path="$git_root/mpc_framework/lib"
    if [ -d "$git_root/c_generated_code" ]; then
        lib_path="$git_root/c_generated_code:$lib_path"
    fi
    export LD_LIBRARY_PATH="${lib_path}:${LD_LIBRARY_PATH:-}"

    # Unset Python-related environments to ensure colcon uses system Python
    unset VIRTUAL_ENV
    unset CONDA_PREFIX
    unset CONDA_SHLVL
    unset CONDA_DEFAULT_ENV
    unset CONDA_PROMPT_MODIFIER
    unset PYTHONPATH
    unset PYTHONDONTWRITEBYTECODE

    # Remove virtual environment and conda paths from PATH
    # Remove paths containing 'env', 'venv', '.virtualenv', 'conda', 'anaconda', 'miniconda'
    local cleaned_path=""
    local IFS=':'
    for dir in $PATH; do
        # Skip if directory contains virtual environment indicators
        if [[ "$dir" =~ (env|venv|\.virtualenv|conda|anaconda|miniconda|IsaacLab) ]]; then
            continue
        fi
        if [ -n "$cleaned_path" ]; then
            cleaned_path="$cleaned_path:$dir"
        else
            cleaned_path="$dir"
        fi
    done
    export PATH="$cleaned_path"

    # Source system ROS2 environment first (for building)
    local ros_setup="/opt/ros/humble/setup.bash"
    if [ -f "$ros_setup" ]; then
        source "$ros_setup"
    fi

    # Source ROS2 workspace setup if available (overrides system)
    local workspace_setup="$git_root/install/setup.bash"
    if [ -f "$workspace_setup" ]; then
        source "$workspace_setup"
    fi
}

# === Display Functions ===

ap_pnc_show_env() {
    echo -e "${AP_PNC_COLOR_BOLD}AP-PnC Environment${AP_PNC_COLOR_RESET}"
    echo -e "${AP_PNC_COLOR_BLUE}  AP_PNC_DIR: ${AP_PNC_DIR}${AP_PNC_COLOR_RESET}"
    echo -e "${AP_PNC_COLOR_GREEN}  Ready ${AP_PNC_CHECKMARK}${AP_PNC_COLOR_RESET}"
    echo
}

# === Auto-setup on source ===
ap_pnc_setup_env
