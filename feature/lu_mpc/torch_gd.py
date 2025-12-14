import torch
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
plt.rcParams['axes.labelsize'] = 8 
# 刻度标签 (Tick Labels) (可选，如果您也想让刻度上的数字变小)
plt.rcParams['xtick.labelsize'] = 8 
plt.rcParams['ytick.labelsize'] = 8 
# 图表标题 (Title) (可选)
plt.rcParams['axes.titlesize'] = 8
# 设置设备
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
torch.set_default_dtype(torch.float64) # 保持与MATLAB精度一致

# --- 1. 定义权重系数 (Weights) ---
# 注意：MATLAB是列向量，这里我们定义为一维 Tensor
kCL = torch.tensor([-2.8465, 23.1516, -0.7335, 0.1407, 0.0930, 3.8276, -0.0006, 3.1951, -0.0289, 0.6425, 0.0164, 0.0031,
                    0.6174, -0.1286, 0.0342, -0.5109, 0.0982, -0.0006, 0.1985, -0.0070, 0.0410, 0.0037, -4.3496, 14.1041], device=device)

kCD = torch.tensor([-0.5908, -1.8161, -0.2045, -0.0036, -0.2861, -0.8862, -0.0590, -0.9215, 0.0130, -0.0035, 0.4958, -0.0032,
                    0.0762, 0.1533, -0.0009, 0.1891, 0.0159, 0.0172, 0.2510, 0.0032, -0.1227, 0.1740, 0.7917, -4.4927], device=device)

kCY = torch.tensor([-0.0678, 0.1151, -0.1024, 0.6002, 0.5367, -0.0922, -0.8523, -0.5220, -0.1006, 0.2266, -0.0345, -0.0768, 0.0748], device=device)

kCLL = torch.tensor([-0.0240, 0.0172, -0.0219, -0.0085, -0.2163, -0.0143, -0.0152, -0.1874, -0.0021, -0.0065, -0.0110, 0.0029,
                     -0.0184, -0.0028, 0.0203, -0.0158, -0.0245, -0.0068], device=device)

kCm = torch.tensor([-1.9963, -2.5059, 0.5987, 0.0120, 0.7977, 0.0543, 0.6310, -0.1122, -0.0798, -0.2177, -0.1059, 0.0029,
                    -0.1388, 0.0132, -0.2605, 0.0015, -0.5789, 0.7902], device=device)

kCn = torch.tensor([0.0681, -0.0338, 0.0054, 0.0028, -0.0052, 0.0034, -0.0048, 0.0217, 0.0111, 0.0121, 0.0685, 0.0240], device=device)


# --- 2. 辅助函数 ---

def deg2rad(deg):
    return deg * np.pi / 180.0

def logistic_func(alpha, alpha1, k1, alpha2, k2):
    """
    PyTorch 版本的 logistic 权重函数。
    alpha: Tensor [N]
    输出: Tensor [N, 1] 用于广播乘法
    """
    # 扩展维度以支持广播: [N] -> [N, 1]
    alpha = alpha.unsqueeze(-1)
    
    # 避免无效输入检查 (PyTorch通常不报error，而是产生nan，这里假设输入合法)
    # P1 = 1./(1 + exp(k1*(alpha - alpha1)))
    # P2 = 1./(1 + exp(-k2*(alpha - alpha2)))
    # yval = 1 - P1 - P2
    
    # 使用 sigmoid 更加数值稳定 (sigmoid(x) = 1 / (1 + exp(-x)))
    # 1/(1+exp(k*(a-a1))) = sigmoid(-k*(a-a1))
    
    P1 = torch.sigmoid(-k1 * (alpha - alpha1))
    P2 = torch.sigmoid(k2 * (alpha - alpha2)) # 注意这里 exp(-k2...) 对应 sigmoid(k2...)
    
    yval = 1.0 - P1 - P2
    return yval

# --- 3. 回归矩阵计算函数 (Regression Matrix Functions) ---
# 所有函数输入 alpha, beta 为 Tensor [N], 输出 reg 为 Tensor [N, feature_dim]

def compute_trig_vars(alpha, beta):
    # 统一变量名，MATLAB代码中有个奇怪的交换：
    # a = alpha; alpha = beta; beta = a;
    # 这意味着传入的第一个参数实际上被当做 beta 用，第二个当做 alpha 用。
    # 为了保持逻辑一致，我们在 Python 函数内部直接交换变量名。
    
    # real_alpha = beta
    # real_beta = alpha
    real_alpha = alpha
    real_beta = beta
    sa = torch.sin(real_alpha)
    ca = torch.cos(real_alpha)
    sb = torch.sin(real_beta)
    cb = torch.cos(real_beta)
    
    return real_alpha, real_beta, sa, ca, sb, cb

def reg_CL(alpha_in, beta_in):
    # 交换变量：alpha_in 对应 MATLAB 的 alpha (实为 beta), beta_in 对应 MATLAB 的 beta (实为 alpha)
    # 这里的逻辑与 MATLAB 代码完全保持一致的数学变换
    alpha, beta, sa, ca, sb, cb = compute_trig_vars(alpha_in, beta_in)
    
    sa2, ca2 = sa**2, ca**2
    sb2, cb2 = sb**2, cb**2
    sa3, ca3 = sa**3, ca**3
    
    # 注意：comp 向量在 MATLAB 是 [features, 1]，这里我们需要 [N, features]
    # 所以构造 list 后使用 stack(dim=1)
    
    # comp_B = [cb2, alpha*cb2]'
    comp_B = torch.stack([cb2, alpha * cb2], dim=1)
    
    # comp_N elements
    comp_N = torch.stack([
        cb2 * ca3,
        ca * sb2,
        ca * cb2 * sa2,
        sa * ca2 * cb2,
        sa * sb2,
        sa3 * cb2,
        torch.abs(sb) * cb * ca2,
        sa * ca * cb * torch.abs(sb),
        torch.abs(sb) * cb * sa2
    ], dim=1)
    
    # Weights & Regressors
    w_B1 = logistic_func(alpha, deg2rad(-190), 1, deg2rad(-170), 1)
    comp_B1 = torch.stack([cb2, (alpha + deg2rad(180)) * cb2], dim=1)
    reg_B1 = w_B1 * comp_B1
    
    w_N11 = logistic_func(alpha, deg2rad(-170), 1, deg2rad(-20), 0.5)
    reg_N11 = w_N11 * comp_N
    
    w_B2 = logistic_func(alpha, deg2rad(-11), 20, deg2rad(18), 10)
    reg_B2 = w_B2 * comp_B
    
    w_N2 = logistic_func(alpha, deg2rad(18), 10, deg2rad(170), 1)
    reg_N2 = w_N2 * comp_N
    
    w_B3 = logistic_func(alpha, deg2rad(170), 1, deg2rad(190), 1)
    comp_B3 = torch.stack([cb2, (alpha - deg2rad(180)) * cb2], dim=1)
    reg_B3 = w_B3 * comp_B3
    
    # Concatenate features: [N, sum_of_features]
    return torch.cat([reg_B1, reg_N11, reg_B2, reg_N2, reg_B3], dim=1)

def reg_CD(alpha_in, beta_in):
    alpha, beta, sa, ca, sb, cb = compute_trig_vars(alpha_in, beta_in)
    
    sa2, ca2 = sa**2, ca**2
    sb2, cb2 = sb**2, cb**2
    sa3, ca3 = sa**3, ca**3
    
    comp_B = torch.stack([cb2, alpha * cb2], dim=1)
    
    comp_N = torch.stack([
        cb2 * ca3,
        ca * sb2,
        ca * cb2 * sa2,
        sa * ca2 * cb2,
        sa * sb2,
        sa3 * cb2,
        torch.abs(sb) * cb * ca2,
        sa * ca * cb * torch.abs(sb),
        torch.abs(sb) * cb * sa2
    ], dim=1)

    w_B1 = logistic_func(alpha, deg2rad(-190), 1, deg2rad(-170), 1)
    comp_B1 = torch.stack([cb2, (alpha + deg2rad(180)) * cb2], dim=1)
    reg_B1 = w_B1 * comp_B1

    w_N1 = logistic_func(alpha, deg2rad(-170), 1, deg2rad(-10), 0.5)
    reg_N1 = w_N1 * comp_N

    w_B2 = logistic_func(alpha, deg2rad(-11), 20, deg2rad(18), 10)
    reg_B2 = w_B2 * comp_B

    w_N2 = logistic_func(alpha, deg2rad(18), 10, deg2rad(170), 1)
    reg_N2 = w_N2 * comp_N

    w_B3 = logistic_func(alpha, deg2rad(170), 1, deg2rad(190), 1)
    comp_B3 = torch.stack([cb2, (alpha - deg2rad(180)) * cb2], dim=1)
    reg_B3 = w_B3 * comp_B3

    return torch.cat([reg_B1, reg_N1, reg_B2, reg_N2, reg_B3], dim=1)

def reg_CY(alpha_in, beta_in):
    alpha, beta, sa, ca, sb, cb = compute_trig_vars(alpha_in, beta_in)
    sb2, cb2 = sb**2, cb**2
    signb = torch.sign(beta) # 注意这里用的是 beta (原代码中的beta，即第一个参数)
    # 等等，compute_trig_vars 已经交换了。
    # 原代码: signb = sign(beta); 这里的 beta 是输入参数交换后的 beta (原来的 alpha)。
    # 让我们仔细看 MATLAB:
    # function [ reg ] = reg_CY( alpha, beta )
    # a = alpha; alpha = beta; beta = a;
    # ...
    # signb = sign(beta); -> 这里使用的是交换后的 beta。
    # 在 compute_trig_vars 中，real_beta 就是交换后的 beta。
    
    comp_N = torch.stack([
        signb * sb2,
        sb * cb * ca,
        sb * cb * sa
    ], dim=1)

    w_B1 = logistic_func(alpha, deg2rad(-190), 1, deg2rad(-170), 1)
    comp_B1 = torch.stack([cb2, (alpha + deg2rad(180)) * cb2], dim=1)
    reg_B1 = w_B1 * comp_B1

    w_N11 = logistic_func(alpha, deg2rad(-170), 1, deg2rad(-90), 1)
    reg_N11 = w_N11 * comp_N

    w_N12 = logistic_func(alpha, deg2rad(-90), 1, deg2rad(-11), 1)
    reg_N12 = w_N12 * comp_N

    w_N2 = logistic_func(alpha, deg2rad(20), 1, deg2rad(170), 1)
    reg_N2 = w_N2 * comp_N

    w_B3 = logistic_func(alpha, deg2rad(170), 1, deg2rad(190), 1)
    comp_B3 = torch.stack([cb2, (alpha - deg2rad(180)) * cb2], dim=1)
    reg_B3 = w_B3 * comp_B3

    return torch.cat([reg_B1, reg_N11, reg_N12, reg_N2, reg_B3], dim=1)

def reg_Cll(alpha_in, beta_in):
    alpha, beta, sa, ca, sb, cb = compute_trig_vars(alpha_in, beta_in)
    sb2, cb2 = sb**2, cb**2
    signb = torch.sign(beta)
    
    comp_B = torch.stack([cb2, alpha * cb2], dim=1)
    comp_N = torch.stack([
        signb * sb2,
        sb * cb * ca,
        sb * cb * sa
    ], dim=1)

    w_B1 = logistic_func(alpha, deg2rad(-190), 1, deg2rad(-170), 1)
    comp_B1 = torch.stack([cb2, (alpha + deg2rad(180)) * cb2], dim=1)
    reg_B1 = w_B1 * comp_B1

    w_N11 = logistic_func(alpha, deg2rad(-170), 1, deg2rad(-90), 5)
    reg_N11 = w_N11 * comp_N

    w_N12 = logistic_func(alpha, deg2rad(-90), 5, deg2rad(-20), 0.5)
    reg_N12 = w_N12 * comp_N

    w_B2 = logistic_func(alpha, deg2rad(-11), 20, deg2rad(18), 10)
    reg_B2 = w_B2 * comp_B

    w_N22 = logistic_func(alpha, deg2rad(18), 10, deg2rad(90), 5)
    reg_N22 = w_N22 * comp_N

    w_N2 = logistic_func(alpha, deg2rad(90), 5, deg2rad(170), 1)
    reg_N2 = w_N2 * comp_N

    w_B3 = logistic_func(alpha, deg2rad(170), 1, deg2rad(190), 1)
    comp_B3 = torch.stack([cb2, (alpha - deg2rad(180)) * cb2], dim=1)
    reg_B3 = w_B3 * comp_B3

    return torch.cat([reg_B1, reg_N11, reg_N12, reg_B2, reg_N2, reg_N22, reg_B3], dim=1)

def reg_Cm(alpha_in, beta_in):
    alpha, beta, sa, ca, sb, cb = compute_trig_vars(alpha_in, beta_in)
    sa2, ca2 = sa**2, ca**2
    sb2, cb2 = sb**2, cb**2
    
    comp_B = torch.stack([cb2, alpha * cb2], dim=1)
    comp_N = torch.stack([
        cb2 * ca2,
        sb2,
        cb2 * sa2,
        torch.abs(sb) * cb * ca,
        sa * ca * cb2,
        torch.abs(sb) * cb * sa
    ], dim=1)

    w_B1 = logistic_func(alpha, deg2rad(-190), 1, deg2rad(-170), 1)
    comp_B1 = torch.stack([cb2, (alpha + deg2rad(180)) * cb2], dim=1)
    reg_B1 = w_B1 * comp_B1

    w_N11 = logistic_func(alpha, deg2rad(-170), 1, deg2rad(-20), 0.5)
    reg_N11 = w_N11 * comp_N

    w_B2 = logistic_func(alpha, deg2rad(-11), 20, deg2rad(18), 10)
    reg_B2 = w_B2 * comp_B

    w_N2 = logistic_func(alpha, deg2rad(18), 10, deg2rad(170), 1)
    reg_N2 = w_N2 * comp_N

    w_B3 = logistic_func(alpha, deg2rad(170), 1, deg2rad(190), 1)
    comp_B3 = torch.stack([cb2, (alpha - deg2rad(180)) * cb2], dim=1)
    reg_B3 = w_B3 * comp_B3

    return torch.cat([reg_B1, reg_N11, reg_B2, reg_N2, reg_B3], dim=1)

def reg_Cn(alpha_in, beta_in):
    alpha, beta, sa, ca, sb, cb = compute_trig_vars(alpha_in, beta_in)
    sb2, cb2 = sb**2, cb**2
    signb = torch.sign(beta)
    
    comp_B = torch.stack([cb2, alpha * cb2], dim=1)
    comp_N = torch.stack([
        signb * sb2,
        sb * cb * ca,
        sb * cb * sa
    ], dim=1)
    
    w_B1 = logistic_func(alpha, deg2rad(-190), 1, deg2rad(-170), 1)
    comp_B1 = torch.stack([cb2, (alpha + deg2rad(180)) * cb2], dim=1)
    reg_B1 = w_B1 * comp_B1
    
    w_N1 = logistic_func(alpha, deg2rad(-170), 1, deg2rad(-10), 10)
    reg_N1 = w_N1 * comp_N
    
    w_B2 = logistic_func(alpha, deg2rad(-10), 10, deg2rad(18), 10)
    reg_B2 = w_B2 * comp_B
    
    w_N2 = logistic_func(alpha, deg2rad(0), 0.5, deg2rad(170), 1)
    reg_N2 = w_N2 * comp_N
    
    w_B3 = logistic_func(alpha, deg2rad(170), 1, deg2rad(190), 1)
    comp_B3 = torch.stack([cb2, (alpha - deg2rad(180)) * cb2], dim=1)
    reg_B3 = w_B3 * comp_B3
    
    return torch.cat([reg_B1, reg_N1, reg_B2, reg_N2, reg_B3], dim=1)


# --- 4. 批量计算与主流程 ---

def main(azimuth=-127.5):
    # Grid definition
    alpha_deg = torch.arange(-90, 91, 1.0, device=device)
    beta_deg = torch.arange(-90, 91, 1.0, device=device)
    
    # Meshgrid (Indexing='ij' corresponds to MATLAB meshgrid behavior partially, 
    # but MATLAB default is 'xy'. Let's stick to standard pytorch meshgrid and adjust shape later)
    # alpha 对应行索引(i), beta 对应列索引(j) -> shape (len_alpha, len_beta)
    ALPHA_grid, BETA_grid = torch.meshgrid(deg2rad(alpha_deg), deg2rad(beta_deg), indexing='ij')
    
    # Flatten for batch processing -> [N]
    alpha_flat = ALPHA_grid.flatten()
    beta_flat = BETA_grid.flatten()
    
    # Batch Compute
    # Each reg_X returns [N, n_features]
    # Multiply with kX [n_features] -> [N]
    
    # matmul automatically handles dot product for vectors: (N, F) @ (F) -> (N)
    CL_flat  = reg_CL(alpha_flat, beta_flat) @ kCL
    CD_flat  = reg_CD(alpha_flat, beta_flat) @ kCD
    CY_flat  = reg_CY(alpha_flat, beta_flat) @ kCY
    CLL_flat = reg_Cll(alpha_flat, beta_flat) @ kCLL
    Cm_flat  = reg_Cm(alpha_flat, beta_flat) @ kCm
    Cn_flat  = reg_Cn(alpha_flat, beta_flat) @ kCn
    
    # Combine into [N, 6]
    # Result tensor: columns are [CL, CD, CY, CLL, Cm, Cn]
    results_flat = torch.stack([CL_flat, CD_flat, CY_flat, CLL_flat, Cm_flat, Cn_flat], dim=1)
    
    print(f"Computation complete. Result shape: {results_flat.shape}")
    
    # --- 5. 可视化 (matplotlib) ---
    # Reshape back to grid for plotting
    shape = ALPHA_grid.shape
    CL = CL_flat.reshape(shape).cpu().numpy()
    CD = CD_flat.reshape(shape).cpu().numpy()
    CY = CY_flat.reshape(shape).cpu().numpy()
    CLL = CLL_flat.reshape(shape).cpu().numpy()
    Cm = Cm_flat.reshape(shape).cpu().numpy()
    Cn = Cn_flat.reshape(shape).cpu().numpy()
    
    X = ALPHA_grid.cpu().numpy() * 180 / np.pi
    Y = BETA_grid.cpu().numpy() * 180 / np.pi
    
    # Plotting
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle("Aerodynamic Coefficients Surface Plots", fontsize=16)
    
    # Layout definition (3 rows, 2 cols)
    plots = [
        (1, CL, r'$C_L$', 'Lift Coefficient'),
        (2, CLL, r'$C_l$', 'Rolling Moment Coefficient'),
        (3, CY, r'$C_Y$', 'Side Force Coefficient'),
        (4, Cm, r'$C_m$', 'Pitching Moment Coefficient'),
        (5, CD, r'$C_D$', 'Drag Coefficient'),
        (6, Cn, r'$C_n$', 'Yawing Moment Coefficient')
    ]
    
    for idx, data, zlabel, title_text in plots:
        ax = fig.add_subplot(3, 2, idx, projection='3d')
        ax.view_init(elev=30, azim=azimuth)
        # 使用 plot_surface 替代 mesh 以获得更好的渲染效果
        surf = ax.plot_surface(X, Y, data, cmap=cm.viridis, linewidth=0, antialiased=False)
        ax.set_xlabel(r'$\alpha$ [deg]')
        ax.set_ylabel(r'$\beta$ [deg]')
        ax.set_zlabel(zlabel)
        ax.set_title(title_text)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Plot aerodynamic coefficients with customizable azimuth angle.')
    parser.add_argument('--azimuth', type=float, default=-127.5, help='Azimuth angle for 3D view in degrees')
    args = parser.parse_args()
    main(args.azimuth)