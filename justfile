# PNC Tailsitter Justfile
# build top algorithms image

# === DATA GENERATION COMMANDS ===

# Generate CL, CD, CM coefficients data under coordinated flight assumption (beta = 0)
generate-coordinated-data:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.model.sim.gd_model_jax import generate_cl_cd_cm_coordinated_data; generate_cl_cd_cm_coordinated_data()"

# Generate CL, CD, CM coefficients data using CasADi symbolic under coordinated flight assumption (beta = 0)
generate-coordinated-data-casadi:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.model.sim.gd_model_casadi import generate_cl_cd_cm_coordinated_data; generate_cl_cd_cm_coordinated_data()"



# Generate 3D mesh data using JAX (usage: just generate-mesh-data-jax [mesh_size])
generate-mesh-data-jax mesh_size="361":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.model.sim.gd_model_jax import generate_mesh_data_jax; generate_mesh_data_jax(mesh_size={{mesh_size}})"

# Generate 3D mesh data using CasADi symbolic
generate-mesh-data-casadi:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.model.sim.gd_model_casadi import generate_mesh_data; generate_mesh_data()"

# Generate 3D mesh data using PyTorch (usage: just generate-mesh-data-torch [mesh_size])
generate-mesh-data-torch mesh_size="361":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.model.sim.gd_model_torch import generate_mesh_data_torch; generate_mesh_data_torch(mesh_size={{mesh_size}})"


# === PLOTTING COMMANDS ===
# All plotting commands now skip data regeneration by default for faster execution.
# Use the --force or -f flag to regenerate data before plotting when needed.
# Example: just plot-aero --force or just plot-mesh-aero -f 45.0 custom_data.npy

# Plot CL, CD, CM coefficients vs alpha under coordinated flight assumption (beta = 0)
# Usage: just plot-aero [--force|-f] [data_file]
plot-aero force_flag="" data_file="":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"

    # Check for force flag
    if [ "{{force_flag}}" = "--force" ] || [ "{{force_flag}}" = "-f" ]; then
        echo "🔄 Regenerating coordinated data..."
        just generate-coordinated-data
        if [ $# -eq 0 ]; then
            data_file="$AP_PNC_DIR/feature/lu_mpc/.cache/coordinated_aero_data_jax.npy"
        fi
    else
        if [ -z "{{data_file}}" ]; then
            data_file="$AP_PNC_DIR/feature/lu_mpc/.cache/coordinated_aero_data_jax.npy"
        else
            data_file="{{data_file}}"
        fi
    fi

    # Plot using the data file
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.plotting.aero_plots import plot_cl_cd_cm_coordinated; plot_cl_cd_cm_coordinated('$data_file', show_plot=False)"

# Plot all aerodynamic coefficients (CL, CD, CY, Cl, Cm, Cn) across AoA
# Usage: just plot-all-aero [data_file]
# Note: Data generation is no longer available - please provide existing data file
plot-all-aero data_file="all_aero_coeffs_data.npy":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"

    if [ "{{data_file}}" = "all_aero_coeffs_data.npy" ]; then
        data_file="$AP_PNC_DIR/feature/lu_mpc/.cache/all_aero_coeffs_data.npy"
    else
        data_file="{{data_file}}"
    fi

    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.plotting.aero_plots import plot_all_aerodynamic_coeffs; plot_all_aerodynamic_coeffs('$data_file')"

# Plot 3D mesh surface of aerodynamic coefficients across AoA and beta
# Usage: just plot-mesh-aero [azimuth] [data_file]
# Note: Data generation is no longer available - please provide existing data file
plot-mesh-aero azimuth="-127.5" data_file="mesh_aero_data.npy":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"

    if [ "{{data_file}}" = "mesh_aero_data.npy" ]; then
        data_file="$AP_PNC_DIR/feature/lu_mpc/.cache/mesh_aero_data.npy"
    else
        data_file="{{data_file}}"
    fi

    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.plotting.mesh_plots import plot_aerodynamic_mesh; plot_aerodynamic_mesh('$data_file', azimuth='{{azimuth}}')"

# Plot 3D mesh surface of aerodynamic coefficients using PyTorch for faster computation
# Usage: just plot-mesh-aero-torch [--force|-f] [azimuth] [data_file]
plot-mesh-aero-torch force_flag="" azimuth="-127.5" data_file="mesh_aero_data_torch.npy":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"

    # Check for force flag
    if [ "{{force_flag}}" = "--force" ] || [ "{{force_flag}}" = "-f" ]; then
        echo "🔄 Regenerating PyTorch mesh data..."
        just generate-mesh-data-torch
        if [ "{{data_file}}" = "mesh_aero_data_torch.npy" ]; then
            data_file="$AP_PNC_DIR/feature/lu_mpc/.cache/mesh_aero_data_torch.npy"
        else
            data_file="{{data_file}}"
        fi
    else
        if [ "{{data_file}}" = "mesh_aero_data_torch.npy" ]; then
            data_file="$AP_PNC_DIR/feature/lu_mpc/.cache/mesh_aero_data_torch.npy"
        else
            data_file="{{data_file}}"
        fi
    fi

    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.plotting.mesh_plots import plot_aerodynamic_mesh_torch; plot_aerodynamic_mesh_torch('$data_file', azimuth='{{azimuth}}')"

# Plot 3D mesh surface of aerodynamic coefficients using JAX for faster computation
# Usage: just plot-mesh-aero-jax [--force|-f] [azimuth] [data_file]
plot-mesh-aero-jax force_flag="" azimuth="-127.5" data_file="mesh_aero_data_jax.npy":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"

    # Check for force flag
    if [ "{{force_flag}}" = "--force" ] || [ "{{force_flag}}" = "-f" ]; then
        echo "🔄 Regenerating JAX mesh data..."
        just generate-mesh-data-jax
        if [ "{{data_file}}" = "mesh_aero_data_jax.npy" ]; then
            data_file="$AP_PNC_DIR/feature/lu_mpc/.cache/mesh_aero_data_jax.npy"
        else
            data_file="{{data_file}}"
        fi
    else
        if [ "{{data_file}}" = "mesh_aero_data_jax.npy" ]; then
            data_file="$AP_PNC_DIR/feature/lu_mpc/.cache/mesh_aero_data_jax.npy"
        else
            data_file="{{data_file}}"
        fi
    fi

    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.plotting.mesh_plots import plot_aerodynamic_mesh_jax; plot_aerodynamic_mesh_jax('$data_file', azimuth='{{azimuth}}', show_plot=False)"

# Plot 3D mesh surface of aerodynamic coefficients using CasADi symbolic
# Usage: just plot-mesh-aero-casadi [--force|-f]
plot-mesh-aero-casadi force_flag="":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"

    # Check for force flag
    if [ "{{force_flag}}" = "--force" ] || [ "{{force_flag}}" = "-f" ]; then
        echo "🔄 Regenerating CasADi mesh data..."
        just generate-mesh-data-casadi
    fi

    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.plotting.mesh_plots import plot_aerodynamic_mesh_casadi; plot_aerodynamic_mesh_casadi(data_file='$AP_PNC_DIR/feature/lu_mpc/.cache/mesh_aero_data_casadi.npy', show_plot=False)"


# === COMBINED COMMANDS (FOR BACKWARD COMPATIBILITY) ===

# Generate data and plot CL, CD, CM coefficients
plot-aero-full:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    just generate-coordinated-data
    just plot-aero

# Plot all aerodynamic coefficients (data generation no longer available)
plot-all-aero-full:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    echo "⚠️  Data generation commands are no longer available. Please ensure data file exists."
    just plot-all-aero

# Plot 3D mesh surface (data generation no longer available)
plot-mesh-aero-full azimuth="-127.5":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    echo "⚠️  Data generation commands are no longer available. Please ensure data file exists."
    just plot-mesh-aero {{azimuth}}

# Generate data and plot 3D mesh surface (JAX)
plot-mesh-aero-jax-full azimuth="125.5":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    just generate-mesh-data-jax
    just plot-mesh-aero-jax {{azimuth}}

# Generate data and plot 3D mesh surface (CasADi)
plot-mesh-aero-casadi-full:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    just generate-mesh-data-casadi
    just plot-mesh-aero-casadi

# Generate data and plot 3D mesh surface (PyTorch)
plot-mesh-aero-torch-full azimuth="-127.5":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    just generate-mesh-data-torch
    just plot-mesh-aero-torch {{azimuth}}


# Compare performance between JAX and PyTorch implementations
compare-timing:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python feature/lu_mpc/tests/test_timing_comparison.py

# Compare performance with custom parameters (usage: just compare-timing-custom mesh_size num_runs)
compare-timing-custom mesh_size num_runs:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python feature/lu_mpc/tests/test_timing_comparison.py {{mesh_size}} {{num_runs}}

# Benchmark JAX performance only
benchmark-jax:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.model.sim.gd_model_jax import benchmark_jax_timing; benchmark_jax_timing()"

# Benchmark PyTorch performance only
benchmark-torch:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.model.sim.gd_model_torch import benchmark_torch_timing; benchmark_torch_timing()"

# Test vmap implementation for correctness and performance
test-vmap:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python feature/lu_mpc/tests/test_vmap_equivalence.py

# Test JAX trim functionality
test-jax-trim:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python feature/lu_mpc/jax_trim.py

# Test configurable drone model with structured configuration (usage: just test-sim-model)
test-sim-model:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python feature/lu_mpc/tests/test_configurable_model.py

# Test symbolic CasADi aerodynamic model
test-symbolic-casadi:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python feature/lu_mpc/model/sim/gd_model_casadi.py

# Test body-to-stability frame coefficient conversion correctness
test-frame-conversion:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.model.sim.gd_model_casadi import test_frame_conversion; test_frame_conversion()"

# Test GD model simulation with ACADOS integrator
test-sim-gd-model:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.model.sim.sim_gd_model import test_gd_simulation; test_gd_simulation()"

# Simulate and visualize GD model with aerodynamic data collection
# Usage: just sim-viz-gd [sim_time] [collect_aero]
sim-viz-gd sim_time="2.0" collect_aero="true":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python feature/lu_mpc/run_sim_viz.py {{sim_time}} {{collect_aero}}

# Run all tests for lu_mpc feature (usage: just test-all)
test-all:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    echo "🚀 Running all lu_mpc tests..."
    echo ""
    echo "Testing structured configuration..."
    $AP_PNC_DIR/.acados_venv/bin/python feature/lu_mpc/tests/test_configurable_model.py
    echo ""
    echo "Testing vmap equivalence..."
    $AP_PNC_DIR/.acados_venv/bin/python feature/lu_mpc/tests/test_vmap_equivalence.py
    echo ""
    echo "Testing ACADOS ENMPC..."
    cd feature/lu_mpc && $AP_PNC_DIR/.acados_venv/bin/python tests/test_acados_enmpc.py
    echo ""
    echo "🎉 All tests completed!"

# Run JAX trim analysis for aircraft equilibrium
jax-trim:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    export JAX_PLATFORM_NAME=cpu
    $AP_PNC_DIR/.acados_venv/bin/python feature/lu_mpc/jax_trim.py


# Legacy command - now redirects to new pipeline
plot-coordinated-aero:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
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
