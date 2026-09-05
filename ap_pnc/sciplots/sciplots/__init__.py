# sciplots/__init__.py
import os
import matplotlib.pyplot as plt
from .styles_discovery import read_styles_in_folders
from .com_funcs_set_style import *
# 获取 sciplots 的安装路径
sciplots_path = os.path.dirname(os.path.abspath(__file__))
styles_path = os.path.join(sciplots_path, "styles")

# 动态注册所有样式文件
stylesheets = read_styles_in_folders(styles_path)
plt.style.core.update_nested_dict(plt.style.library, stylesheets)
plt.style.core.available[:] = sorted(plt.style.library.keys())

# 可选：暴露接口
__all__ = ['plt']  # 允许其他模块通过 `from sciplots import plt` 使用
