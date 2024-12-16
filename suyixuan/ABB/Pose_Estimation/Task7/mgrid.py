"""
Author: Yixuan Su
Date: 2024/12/10 19:42
File: mgrid.py
Description: 
"""
import numpy as np

# 生成 x 和 y 坐标
x = np.arange(0, 3)
y = np.arange(0, 4)
print(x)
print(y)

# # 创建 2D 网格
# X, Y = np.meshgrid(x, y)

grid = np.mgrid[0:3, 0:4]
# print(X)
# print(Y)
print("Original grid shape:", grid.shape)
print("Original grid shape:", grid)


# 通过转置和 reshape 展平成坐标对
coordinates = grid.T
print("Flattened coordinates:\n", coordinates)

coordinates2 = coordinates.reshape(-1, 2)
print("Flattened coordinates:\n", coordinates2)