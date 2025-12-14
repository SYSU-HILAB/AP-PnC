import jax
import jax.numpy as jnp
from jaxopt import LevenbergMarquardt
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np # Used for saving CSV
from gd_model_jax import aerodynamics_B_vb
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
    Velocity_body = jnp.array([
        V * jnp.cos(alpha) * jnp.cos(beta),
        V * jnp.sin(beta),
        V * jnp.sin(alpha) * jnp.cos(beta)
    ])

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
    W_z =  mass * g * jnp.cos(theta)

    # 4. Residuals
    R_Fx = FORCESX + T + W_x
    R_Fz = FORCESZ + W_z
    R_My = MOMENTY + M_y_thrust
    R_V = V - target_V # Constraint

    return jnp.array([R_Fx, R_Fz, R_My, R_V])

# ==========================================
# Batch Solver Setup
# ==========================================

# 1. Define Range
num_points = 10000
target_velocities = jnp.linspace(0.1, 13.0, num_points) # Start 0.1 to avoid V=0 singularities

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

init_params_batch = jnp.stack([
    init_v_guess, 
    init_alpha_guess, 
    init_thrust_guess, 
    init_moment_guess
], axis=1)

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
results = sol_batch.params # Shape (50, 4)
residuals = sol_batch.state.value # Shape (50,)

# Unpack for plotting
V_res = results[:, 0]
alpha_res_deg = jnp.rad2deg(results[:, 1])
thrust_res = results[:, 2]
moment_res = results[:, 3]

# ==========================================
# Output 1: CSV Generation
# ==========================================

df = pd.DataFrame({
    'Target_V_ms': target_velocities,
    'Trimmed_V_ms': V_res,
    'Alpha_deg': alpha_res_deg,
    'Thrust_N': thrust_res,
    'Moment_Nm': moment_res,
    'Residual_Error': residuals
})

csv_filename = 'trim_results.csv'
df.to_csv(csv_filename, index=False)
print(f"Results saved to {csv_filename}")

# ==========================================
# Output 2: Plotting
# ==========================================

fig, axs = plt.subplots(1, 3, figsize=(18, 5))
fig.suptitle(f'Trim Results (Mass={mass_val}kg)', fontsize=16)

# Plot 1: Alpha vs V
axs[0].plot(V_res, alpha_res_deg, 'b.-', linewidth=2)
axs[0].set_xlabel('Velocity (m/s)')
axs[0].set_ylabel('Alpha (deg)')
axs[0].set_title('Angle of Attack vs Velocity')
axs[0].grid(True)

# Plot 2: Thrust vs V
axs[1].plot(V_res, thrust_res, 'r.-', linewidth=2)
axs[1].set_xlabel('Velocity (m/s)')
axs[1].set_ylabel('Thrust (N)')
axs[1].set_title('Thrust vs Velocity')
axs[1].grid(True)

# Plot 3: Moment vs V
axs[2].plot(V_res, moment_res, 'g.-', linewidth=2)
axs[2].set_xlabel('Velocity (m/s)')
axs[2].set_ylabel('Control Moment (Nm)')
axs[2].set_title('Pitch Moment vs Velocity')
axs[2].grid(True)

plt.tight_layout()
plt.show()