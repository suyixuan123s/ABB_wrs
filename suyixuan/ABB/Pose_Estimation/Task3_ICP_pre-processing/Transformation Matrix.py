"""
Author: Yixuan Su
Date: 2024/12/16 17:50
File: Transformation Matrix.py
Description: 
"""
import numpy as np

# 初始变换矩阵
T_init = np.array([
    [-0.82649164, -0.06160907, 0.55956759, 0.77986891],
    [-0.56273542, 0.06304022, -0.82422981, 1.22546576],
    [0.01550476, -0.99610756, -0.08677182, 0.77429397],
    [0.0, 0.0, 0.0, 1.0]
])

# 绕Y轴旋转180度的旋转矩阵（4x4）
T_y = np.array([
    [-1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, -1, 0],
    [0, 0, 0, 1]
])

# 平移矩阵（4x4）
T_t = np.array([
    [1, 0, 0, 1.1],
    [0, 1, 0, 0.0],
    [0, 0, 1, 0.8],
    [0, 0, 0, 1]
])

# 合成变换矩阵
T_final = T_init @ T_y @ T_t

print("Final Transformation Matrix:")
print(T_final)
