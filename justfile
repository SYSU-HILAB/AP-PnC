# PNC Tailsitter Justfile
# build top algorithms image

# Plot CL, CD, CM coefficients vs alpha under coordinated flight assumption (beta = 0)
plot-aero:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.gd_model_jax import plot_cl_cd_cm_vs_alpha_coordinated; plot_cl_cd_cm_vs_alpha_coordinated()"

# Plot all aerodynamic coefficients (CL, CD, CY, Cl, Cm, Cn) across AoA
plot-all-aero:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.gd_model import plot_all_aerodynamic_coeffs; plot_all_aerodynamic_coeffs()"

# Plot 3D mesh surface of aerodynamic coefficients across AoA and beta (usage: just plot-mesh-aero [azimuth])
plot-mesh-aero azimuth="-127.5":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.gd_model import plot_aerodynamic_mesh; plot_aerodynamic_mesh({{azimuth}})"

# Plot 3D mesh surface of aerodynamic coefficients using PyTorch for faster computation (usage: just plot-mesh-aero-torch [azimuth])
plot-mesh-aero-torch azimuth="-127.5":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.gd_model_torch import plot_aerodynamic_mesh_torch; plot_aerodynamic_mesh_torch({{azimuth}})"

# Plot 3D mesh surface of aerodynamic coefficients using JAX for faster computation (usage: just plot-mesh-aero-jax [azimuth])
plot-mesh-aero-jax azimuth="-127.5":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.gd_model_jax import plot_aerodynamic_mesh_jax; plot_aerodynamic_mesh_jax({{azimuth}})"

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
    $AP_PNC_DIR/.acados_venv/bin/python feature/lu_mpc/test_vmap_equivalence.py

# Test JAX trim functionality
test-jax-trim:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python feature/lu_mpc/jax_trim.py

# Run JAX trim analysis for aircraft equilibrium
jax-trim:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    export JAX_PLATFORM_NAME=cpu
    $AP_PNC_DIR/.acados_venv/bin/python feature/lu_mpc/jax_trim.py

# Plot aerodynamic coefficients using torch_gd.py (usage: just plot-torch-gd [azimuth])
plot-torch-gd azimuth="-127.5":
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python feature/lu_mpc/torch_gd.py --azimuth {{azimuth}}

# Plot CL, CD, CM coefficients vs alpha under coordinated flight assumption (beta = 0)
plot-coordinated-aero:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.gd_model_jax import plot_cl_cd_cm_vs_alpha_coordinated; plot_cl_cd_cm_vs_alpha_coordinated()"

alias p := plot-aero
alias pa := plot-all-aero
alias pm := plot-mesh-aero
alias pt := plot-mesh-aero-torch
alias pj := plot-mesh-aero-jax
alias pg := plot-torch-gd
alias pca := plot-coordinated-aero
alias ct := compare-timing
alias bj := benchmark-jax
alias bt := benchmark-torch
alias tv := test-vmap
alias jt := test-jax-trim
alias jtrim := jax-trim
