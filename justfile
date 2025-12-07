# PNC Tailsitter Justfile
# build top algorithms image

# Plot CL and CD coefficients across AoA
plot-aero:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python feature/lu_mpc/gd_model.py

# Plot all aerodynamic coefficients (CL, CD, CY, Cl, Cm, Cn) across AoA
plot-all-aero:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.gd_model import plot_all_aerodynamic_coeffs; plot_all_aerodynamic_coeffs()"

# Plot 3D mesh surface of aerodynamic coefficients across AoA and beta
plot-mesh-aero:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.gd_model import plot_aerodynamic_mesh; plot_aerodynamic_mesh()"

# Plot 3D mesh surface of aerodynamic coefficients using PyTorch for faster computation
plot-mesh-aero-torch:
    #!/usr/bin/env bash
    export AP_PNC_DIR="/home/rec/AP-PnC"
    $AP_PNC_DIR/.acados_venv/bin/python -c "from feature.lu_mpc.gd_model_torch import plot_aerodynamic_mesh_torch; plot_aerodynamic_mesh_torch()"

alias p := plot-aero
alias pa := plot-all-aero
alias pm := plot-mesh-aero
alias pt := plot-mesh-aero-torch
