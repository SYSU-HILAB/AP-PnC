import numpy as np
import matplotlib.pyplot as plt
plt.switch_backend('Agg')
# 生成一组示例数据，确保数据都是非负的

def plot_density(data):
# 计算众数、四分位数和最大值
  mode = float(np.argmax(np.bincount(data.astype(int))))
  q1, median, q3 = np.percentile(data, [25, 50, 75])
  max_value = np.max(data)

  # 画图
  plt.hist(data, bins=30, density=True, alpha=0.7, color='blue', label='Histogram')
  plt.axvline(mode, color='red', linestyle='dashed', linewidth=2, label='Mode')
  plt.axvline(q1, color='green', linestyle='dashed', linewidth=2, label='Q1')
  plt.axvline(median, color='orange', linestyle='dashed', linewidth=2, label='Median')
  plt.axvline(q3, color='purple', linestyle='dashed', linewidth=2, label='Q3')
  plt.axvline(max_value, color='brown', linestyle='dashed', linewidth=2, label='Max')


  plt.legend()
  plt.title('Statistical Lines in a Histogram')
  plt.xlabel('Value')
  plt.ylabel('Density')
  # plt.show()
  plt.savefig('plot.png')