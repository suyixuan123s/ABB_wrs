"""
Author: Yixuan Su
Date: 2024/12/03 22:01
File: compute_center_of_mass.py
Description: 
"""

import open3d as o3d
import numpy as np

# 计算点云的中心点
def compute_center_of_mass(point_cloud):
    points = np.asarray(point_cloud.points)
    # 计算中心点（所有点的平均坐标）
    center_of_mass = np.mean(points, axis=0)
    return center_of_mass

# 通过变换矩阵转换点云的坐标
def transform_point(point, transformation_matrix):
    # 增加一个1来完成齐次坐标变换
    point_homogeneous = np.append(point, 1)
    transformed_point = np.dot(transformation_matrix, point_homogeneous)
    return transformed_point[:3]  # 返回x, y, z坐标

# 加载已经进行ICP配准后的源点云和目标点云
source = o3d.io.read_point_cloud(r"/suyixuan/ABB/Task1_Point_cloud_capture/reverted_point_cloud.ply")
target = o3d.io.read_point_cloud(r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\ICP\converted_stl_point_cloud.ply")

# 计算源点云的中心点（假设为盒子中心）
center_of_mass_source = compute_center_of_mass(target)
print(f"Source center of mass (before ICP transformation): {center_of_mass_source}")

# 获取ICP配准后的变换矩阵
transformation_matrix = np.array([
    [9.98692787e-01, -4.23793596e-03, -5.09388162e-02, -1.91442033e-02],
    [2.22325195e-04, 9.96907750e-01, -7.85804563e-02, -2.94302326e-02],
    [5.11143198e-02, 7.84664059e-02, 9.95605519e-01, -1.87468042e-02],
    [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])

# 将源点云中心点应用到变换矩阵中，得到中心点在相机坐标系中的位置
transformed_center = transform_point(center_of_mass_source, transformation_matrix)
print(f"Transformed center of mass (in camera coordinates): {transformed_center}")

# 获取旋转矩阵
rotation_matrix = transformation_matrix[:3, :3]
print("Rotation matrix (orientation):")
print(rotation_matrix)

# 位姿信息：位置 + 朝向
position = transformed_center
orientation = rotation_matrix

print(f"Position (camera coordinates): {position}")
print(f"Orientation (rotation matrix):\n{orientation}")

