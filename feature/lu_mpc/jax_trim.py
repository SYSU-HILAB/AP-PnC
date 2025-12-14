import jax
import jax.numpy as jnp
from jaxopt import LevenbergMarquardt
from gd_model_jax import compute_aerodynamic_coeffs_jax

# 假设这是你的全量力矩方程（封装了你代码里的 compute_aerodynamic_coeffs_jax）
@jax.jit
def aircraft_equations(trim_vars, flight_condition):
    alpha = trim_vars[0]
    delta_e = trim_vars[1] # 假设你的模型里加入了舵面效应
    beta = 0.0 # 纵向配平假设
    
    # 1. 调用你的气动函数 (利用你的代码)
    # 注意：你现在的代码只有 alpha, beta 输入，
    # 实际配平需要把 delta_e (舵偏角) 的贡献加进去
    CL, CD, CY, CLL, CM, CN = compute_aerodynamic_coeffs_jax(alpha, beta)
    
    # 假设简单的舵面线性模型 (实际请用你的数据)
    CL_total = CL + 0.1 * delta_e 
    CM_total = CM + 0.5 * delta_e
    
    # 2. 物理方程 (简单举例)
    # 目标：升力=重力，力矩=0
    #这里做归一化处理以便求解器收敛
    velocity = flight_condition['v']
    density = flight_condition['rho']
    S = flight_condition['S']
    m = flight_condition['m']
    g = 9.81
    
    q_bar = 0.5 * density * velocity**2
    
    Lift = CL_total * q_bar * S
    Moment = CM_total * q_bar * S * flight_condition['c']
    
    residual_force = Lift - m * g
    residual_moment = Moment
    
    return jnp.array([residual_force, residual_moment])

# 定义求解器
solver = LevenbergMarquardt(residual_fun=aircraft_equations)

# 运行求解
flight_cond = {'v': 100.0, 'rho': 1.225, 'S': 20.0, 'm': 1000.0, 'c': 1.0}
init_guess = jnp.array([0.1, 0.0]) # alpha=0.1 rad, delta_e=0

# JAX 会自动计算雅可比矩阵来加速收敛
params, state = solver.run(init_guess, flight_condition=flight_cond)

print(f"Trimmed Alpha: {params[0]}, Trimmed Delta_e: {params[1]}")