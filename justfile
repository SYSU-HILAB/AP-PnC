# PNC Tailsitter Justfile
# build top algorithms image

# === DATA GENERATION COMMANDS ===

# Generate CL, CD, CM coefficients data under coordinated flight assumption (beta = 0)
generate-coordinated-data:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.gd_model_jax import generate_cl_cd_cm_coordinated_data; generate_cl_cd_cm_coordinated_data()"

# Generate CL, CD, CM coefficients data using CasADi symbolic under coordinated flight assumption (beta = 0)
generate-coordinated-data-casadi:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.gd_model_casadi import generate_cl_cd_cm_coordinated_data; generate_cl_cd_cm_coordinated_data()"

# Generate all aerodynamic coefficients data
generate-all-aero-data:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.gd_model import generate_all_coeffs_data; generate_all_coeffs_data()"

# Generate 3D mesh data using CasADi (usage: just generate-mesh-data [mesh_size])
generate-mesh-data mesh_size="361":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.gd_model import generate_mesh_data; generate_mesh_data(mesh_size={{mesh_size}})"

# Generate 3D mesh data using JAX (usage: just generate-mesh-data-jax [mesh_size])
generate-mesh-data-jax mesh_size="361":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.gd_model_jax import generate_mesh_data_jax; generate_mesh_data_jax(mesh_size={{mesh_size}})"

# Generate 3D mesh data using CasADi symbolic
generate-mesh-data-casadi:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.gd_model_casadi import generate_mesh_data; generate_mesh_data()"

# Generate 3D mesh data using PyTorch (usage: just generate-mesh-data-torch [mesh_size])
generate-mesh-data-torch mesh_size="361":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.gd_model_torch import generate_mesh_data_torch; generate_mesh_data_torch(mesh_size={{mesh_size}})"

# Generate torch_gd data
generate-torch-gd-data:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.torch_gd import generate_torch_gd_data; generate_torch_gd_data()"

# === PLOTTING COMMANDS ===

# Plot CL, CD, CM coefficients vs alpha under coordinated flight assumption (beta = 0)
plot-aero:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    if [ $# -eq 0 ]; then
        # No arguments - use default data file
        $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.plotting.aero_plots import plot_cl_cd_cm_coordinated; plot_cl_cd_cm_coordinated('$AP_PNC_DIR/feature/lu_mpc/.cache/coordinated_aero_data_jax.npy', show_plot=False)"
    else
        # Use provided data file
        $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.plotting.aero_plots import plot_cl_cd_cm_coordinated; plot_cl_cd_cm_coordinated('$1', show_plot=False)"
    fi

# Plot all aerodynamic coefficients (CL, CD, CY, Cl, Cm, Cn) across AoA
plot-all-aero data_file="all_aero_coeffs_data.npy":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.plotting.aero_plots import plot_all_aerodynamic_coeffs; plot_all_aerodynamic_coeffs('{{data_file}}')"

# Plot 3D mesh surface of aerodynamic coefficients across AoA and beta (usage: just plot-mesh-aero [azimuth] [data_file])
plot-mesh-aero azimuth="-127.5" data_file="mesh_aero_data.npy":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.plotting.mesh_plots import plot_aerodynamic_mesh; plot_aerodynamic_mesh('{{data_file}}', azimuth='{{azimuth}}')"

# Plot 3D mesh surface of aerodynamic coefficients using PyTorch for faster computation (usage: just plot-mesh-aero-torch [azimuth] [data_file])
plot-mesh-aero-torch azimuth="-127.5" data_file="mesh_aero_data_torch.npy":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.plotting.mesh_plots import plot_aerodynamic_mesh_torch; plot_aerodynamic_mesh_torch('{{data_file}}', azimuth='{{azimuth}}')"

# Plot 3D mesh surface of aerodynamic coefficients using JAX for faster computation (usage: just plot-mesh-aero-jax [azimuth] [data_file])
plot-mesh-aero-jax azimuth="125.5" data_file="mesh_aero_data_jax.npy":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.plotting.mesh_plots import plot_aerodynamic_mesh_jax; plot_aerodynamic_mesh_jax('{{data_file}}', azimuth='{{azimuth}}', show_plot=False)"

# Plot 3D mesh surface of aerodynamic coefficients using CasADi symbolic
plot-mesh-aero-casadi:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.plotting.mesh_plots import plot_aerodynamic_mesh_casadi; plot_aerodynamic_mesh_casadi(data_file='$AP_PNC_DIR/feature/lu_mpc/.cache/mesh_aero_data_casadi.npy', show_plot=False)"

# Plot torch_gd aerodynamic coefficients (usage: just plot-torch-gd [azimuth] [data_file])
plot-torch-gd azimuth="-127.5" data_file="torch_gd_data.npy":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.plotting.torch_plots import plot_torch_gd; plot_torch_gd('{{data_file}}', azimuth='{{azimuth}}')"

# === COMBINED COMMANDS (FOR BACKWARD COMPATIBILITY) ===

# Generate data and plot CL, CD, CM coefficients
plot-aero-full:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    just generate-coordinated-data
    just plot-aero

# Generate data and plot all aerodynamic coefficients
plot-all-aero-full:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    just generate-all-aero-data
    just plot-all-aero

# Generate data and plot 3D mesh surface (CasADi)
plot-mesh-aero-full azimuth="-127.5":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    just generate-mesh-data
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

# Generate data and plot torch_gd
plot-torch-gd-full azimuth="-127.5":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    just generate-torch-gd-data
    just plot-torch-gd {{azimuth}}

# Compare performance between JAX and PyTorch implementations
compare-timing:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python feature/lu_mpc/timing_comparison.py

# Compare performance with custom parameters (usage: just compare-timing-custom mesh_size num_runs)
compare-timing-custom mesh_size num_runs:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python feature/lu_mpc/timing_comparison.py {{mesh_size}} {{num_runs}}

# Benchmark JAX performance only
benchmark-jax:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.gd_model_jax import benchmark_jax_timing; benchmark_jax_timing()"

# Benchmark PyTorch performance only
benchmark-torch:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.gd_model_torch import benchmark_torch_timing; benchmark_torch_timing()"

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
    $AP_PNC_DIR/.acados_venv/bin/python feature/lu_mpc/gd_model_casadi.py

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

# Legacy torch_gd direct plotting (usage: just plot-torch-gd-legacy [azimuth])
plot-torch-gd-legacy azimuth="-127.5":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python feature/lu_mpc/torch_gd.py --azimuth {{azimuth}}

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
alias pg := plot-torch-gd
alias pca := plot-coordinated-aero
alias ct := compare-timing
alias bj := benchmark-jax
alias bt := benchmark-torch
alias tv := test-vmap
alias jt := test-jax-trim
alias jtrim := jax-trim
alias tsm := test-sim-model
alias tsc := test-symbolic-casadi
alias ta := test-all
