from pathlib import Path

import jax
import jax.numpy as jnp
import matplotlib.pyplot as plt
import pandas as pd
from jaxopt import LevenbergMarquardt

from tooling.env import project_path

# Use pgf backend for publication-quality plots
plt.rcParams.update({
    "pgf.texsystem": "pdflatex",
    "font.family": "serif",
    "font.serif": [],
    "text.usetex": True,
    "pgf.preamble": r"\usepackage{amsmath}",
})

from tooling.paper.methods.lu.gd_model_jax import (  # noqa: E402 -- after rcParams/pgf setup
    aerodynamics_B_vb,
)

# -------------------------------------------------------------------------------
# MOCK IMPORT: Replace this block with your actual import
# from gd_model_jax import aerodynamics_B_vb


@jax.jit
def aircraft_equations(trim_vars, target_V, mass, g):
    """
    Computes residuals.
    Modified signature: explicitly accepts target_V, mass, and g
    to make vmapping over target_V easier.
    """
    # Unpack Trim variables
    V = trim_vars[0]
    alpha = trim_vars[1]
    T = trim_vars[2]
    M_y_thrust = trim_vars[3]

    beta = 0.0

    # 1. State Construction
    Velocity_body = jnp.array(
        [
            V * jnp.cos(alpha) * jnp.cos(beta),
            V * jnp.sin(beta),
            V * jnp.sin(alpha) * jnp.cos(beta),
        ]
    )

    # 2. Aerodynamics
    # Since aerodynamics_B_vb supports batching but here we are inside a
    # function processed by JAX (either single or inside vmap),
    # we treat this as a single instance operation.
    FORCES, MOMENTS, _, _ = aerodynamics_B_vb(Velocity_body)

    FORCESX = FORCES[0]
    FORCESZ = FORCES[2]
    MOMENTY = MOMENTS[1]

    # 3. Physics (Level Flight)
    theta = alpha

    W_x = -mass * g * jnp.sin(theta)
    W_z = mass * g * jnp.cos(theta)

    # 4. Residuals
    R_Fx = FORCESX + T + W_x
    R_Fz = FORCESZ + W_z
    R_My = MOMENTY + M_y_thrust
    R_V = V - target_V  # Constraint

    return jnp.array([R_Fx, R_Fz, R_My, R_V])


# ==========================================
# Batch Solver Setup
# ==========================================

# 1. Define Range
num_points = 10000
target_velocities = jnp.linspace(
    0.1, 13.0, num_points
)  # Start 0.1 to avoid V=0 singularities

# 2. Define Constants
mass_val = 2.0
g_val = 9.81

# 3. Create Initial Guesses (Batch)
# Shape: (50, 4)
# We vary the initial V guess to match the target V to help the solver.
# We keep alpha guess constant (e.g., 10 deg) or vary it if you expect high alpha at low speed.
init_v_guess = target_velocities
init_alpha_guess = jnp.ones(num_points) * jnp.deg2rad(50.0)
init_thrust_guess = jnp.ones(num_points) * 10.0
init_moment_guess = jnp.zeros(num_points)

init_params_batch = jnp.stack(
    [init_v_guess, init_alpha_guess, init_thrust_guess, init_moment_guess], axis=1
)
# from jaxopt import LevenbergMarquardt

# 4. Instantiate Solver
# We set implicit_diff=False generally for pure minimization,
# though default is usually fine.
lm = LevenbergMarquardt(residual_fun=aircraft_equations)

# 5. Define a Batched Run Function
# We use jax.vmap to vectorize the solver.run method.
# We map over:
#   Arg 0: init_params (batch dimension 0)
#   Kwarg target_V: (batch dimension 0)
#   Kwarg mass: (None - do not map, it's constant)
#   Kwarg g: (None - do not map, it's constant)


def run_solver_single(init_p, t_v):
    return lm.run(init_p, target_V=t_v, mass=mass_val, g=g_val)


print(f"Running Batch Trim for {num_points} points...")
batch_run = jax.vmap(run_solver_single)
sol_batch = batch_run(init_params_batch, target_velocities)

# Extract results
results = sol_batch.params  # Shape (50, 4)
residuals = sol_batch.state.value  # Shape (50,)

# Unpack for plotting
V_res = results[:, 0]
alpha_res_deg = jnp.rad2deg(results[:, 1])
thrust_res = results[:, 2]
moment_res = results[:, 3]

# ==========================================
# Output 1: CSV Generation
# ==========================================

df = pd.DataFrame(
    {
        "Target_V_ms": target_velocities,
        "Trimmed_V_ms": V_res,
        "Alpha_deg": alpha_res_deg,
        "Thrust_N": thrust_res,
        "Moment_Nm": moment_res,
        "Residual_Error": residuals,
    }
)

# Get git root for artifact paths
git_root = Path(__file__).resolve().parent.parent.parent.parent
figs_dir = project_path(".artifacts/paper/figs")
figs_dir.mkdir(parents=True, exist_ok=True)
csv_filename = figs_dir / "trim_results.csv"
df.to_csv(csv_filename, index=False)
print(f"Results saved to {csv_filename}")

# ==========================================
# Output 2: Plotting
# ==========================================

fig, ax1 = plt.subplots(1, 1, figsize=(10, 5))

# Plot 1: Velocity vs Alpha (left y-axis, blue solid)
ax1.set_xlabel(r"$\alpha$ (deg)", fontsize=19)
ax1.set_ylabel("Velocity (m/s)", color="blue", fontsize=12)
line1 = ax1.plot(alpha_res_deg, V_res, "b-", linewidth=2, label="Velocity")
ax1.tick_params(axis="y", labelcolor="blue")
ax1.grid(True, alpha=0.3)

# Plot 2: Thrust vs Alpha (right y-axis, dashed red)
ax2 = ax1.twinx()
ax2.set_ylabel("Thrust (N)", color="red", fontsize=12)
line2 = ax2.plot(alpha_res_deg, thrust_res, "r--", linewidth=2, label="Thrust")
ax2.tick_params(axis="y", labelcolor="red")

# Combine legends
lines = line1 + line2
labels = [line.get_label() for line in lines]
legend = ax1.legend(lines, labels, loc="upper center", bbox_to_anchor=(0.5, -0.15))

pdf_filename = figs_dir / "trim_results.pdf"
plt.savefig(pdf_filename, backend="pgf", bbox_inches="tight")
print(f"Plot saved to {pdf_filename}")
plt.show()
