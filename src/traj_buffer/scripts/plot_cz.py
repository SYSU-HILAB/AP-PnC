import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
# 定义fit_data命名空间中的变量和函数
class FitData:
    def __init__(self):
        self.w = 1.0
        self.a0 = -11.8
        self.a1 = -1.099
        self.b1 = 22.35
        self.a2 = 13.05
        self.b2 = 1.314
        self.a3 = 0.788
        self.b3 = -5.099
        self.a4 = -1.015
        self.b4 = -0.2115
        self.kPho = 1.225
        self.kS = 0.25

# 初始化fit_data
fit_data = FitData()

def get_Cz_from_alpha(alpha):
    if alpha < 0.0:
        print("Sing")
        return 0.0

    # fourier series about alpha
    cz = (fit_data.a0 + fit_data.a1 * np.cos(fit_data.w * alpha) + fit_data.b1 * np.sin(fit_data.w * alpha) +
          fit_data.a2 * np.cos(2 * fit_data.w * alpha) + fit_data.b2 * np.sin(2 * fit_data.w * alpha) +
          fit_data.a3 * np.cos(3 * fit_data.w * alpha) + fit_data.b3 * np.sin(3 * fit_data.w * alpha) +
          fit_data.a4 * np.cos(4 * fit_data.w * alpha) + fit_data.b4 * np.sin(4 * fit_data.w * alpha))

    mass = 1.8
    # factor using rho and area
    cz_rectified = 0.5 * fit_data.kPho * fit_data.kS * cz / mass
    return cz_rectified

# 定义alpha的范围并计算对应的Cz值
alpha_values = np.linspace(0, np.radians(120), 500)
cz_values = [get_Cz_from_alpha(alpha) for alpha in alpha_values]

# Initialize an empty DataFrame
columns = ['pitch', 'alpha', 'acc_z', 'cz']
df_real = pd.DataFrame(columns=columns)
new_data = pd.DataFrame([{'pitch': 0.3490658503988659, 'alpha': 0.0, 'acc_z': 9.8, 'cz': 0.3},
                         {'pitch': 0.787144, 'alpha': 0.0, 'acc_z': 9.7, 'cz': 0.134},
                         {'pitch': 0.964268, 'alpha': 0.0, 'acc_z': 9.7, 'cz': 0.10872505709878282},
                         {'pitch': 1.045764, 'alpha': 0.0, 'acc_z': 7.019415, 'cz': 0.08899869684205915},
                         ])
df_real = pd.concat([df_real,new_data],ignore_index=True)
df_real['alpha'] = np.pi/2 - df_real['pitch']
print(df_real)
# 绘制Cz与alpha的关系图
plt.figure(figsize=(10, 6))
plt.plot(np.degrees(alpha_values), cz_values, label='Cz vs Alpha')
plt.scatter(df_real['alpha']*180/np.pi, df_real['cz'], color='red', marker='*', s=32, label='real-cz')
plt.xlabel('Alpha (degrees)')
plt.ylabel('Cz')
plt.title('Cz vs Alpha')
plt.legend()
plt.grid(True)
plt.show()
