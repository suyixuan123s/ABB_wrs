"""
Author: Yixuan Su
Date: 2024/11/17 10:26
File: get_transformation_matrices_camera_to_sim.py
"""

import os
import numpy as np


def compute_transformation(alpha_deg, beta_deg, gamma_deg, tx, ty, tz):
    # 将角度转换为弧度
    alpha = np.radians(alpha_deg)
    beta = np.radians(beta_deg)
    gamma = np.radians(gamma_deg)

    # 构造旋转矩阵 (X, Y, Z 轴)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(alpha), -np.sin(alpha)],
        [0, np.sin(alpha), np.cos(alpha)]
    ])

    Ry = np.array([
        [np.cos(beta), 0, np.sin(beta)],
        [0, 1, 0],
        [-np.sin(beta), 0, np.cos(beta)]
    ])

    Rz = np.array([
        [np.cos(gamma), -np.sin(gamma), 0],
        [np.sin(gamma), np.cos(gamma), 0],
        [0, 0, 1]
    ])

    # 组合旋转矩阵
    rotation_matrix = Rz @ Ry @ Rx

    # 构造齐次变换矩阵 (相机到仿真坐标系的外参)
    transformation_camera_to_sim = np.eye(4)
    transformation_camera_to_sim[:3, :3] = rotation_matrix
    transformation_camera_to_sim[:3, 3] = [tx, ty, tz]

    # # 计算仿真坐标系到相机坐标系的外参 (取逆)
    # transformation_sim_to_camera = np.linalg.inv(transformation_camera_to_sim)

    # return transformation_camera_to_sim, transformation_sim_to_camera
    return transformation_camera_to_sim


# 已知参数
# alpha, beta, gamma = -58.8, -3.0, -179.0
# tx, ty, tz = 0.47, 0.757, 1.27


# alpha, beta, gamma = -148.0, -0.4, -178.0
# tx, ty, tz = 0.525, 0.76, 1.25

alpha, beta, gamma = 27.5, -180.0, 180.0
tx, ty, tz = 0.42, -0.77, 1.23
# 计算外参矩阵
transformation_camera_to_sim = compute_transformation(alpha, beta, gamma, tx, ty, tz)

# 指定文件路径
file_path = r"/suyixuan/ABB/Pose_Estimation/Task2_Finding_transformation_matrix/transformation_matrices.txt"

# 检查目录是否存在，如果不存在则创建
os.makedirs(os.path.dirname(file_path), exist_ok=True)

# 将结果保存到文件
with open(file_path, "w") as f:
    f.write("相机到仿真坐标系的外参 (Camera to Simulation):\n")
    f.write(np.array2string(transformation_camera_to_sim, formatter={'float_kind': lambda x: f"{x:.6f}"}))
    # f.write("\n\n仿真坐标系到相机坐标系的外参 (Simulation to Camera):\n")
    # f.write(np.array2string(transformation_sim_to_camera, formatter={'float_kind': lambda x: f"{x:.6f}"}))

print(f"文件已保存到: {file_path}")
