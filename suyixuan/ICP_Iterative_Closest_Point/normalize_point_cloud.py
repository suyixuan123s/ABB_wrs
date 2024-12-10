"""
Author: Yixuan Su
Date: 2024/12/03 21:17
File: normalize_point_cloud.py
Description: 
"""

import open3d as o3d
import numpy as np

# 1. 从 STL 文件生成点云
def stl_to_point_cloud(stl_file, num_points):
    # 读取 STL 文件并生成网格
    mesh = o3d.io.read_triangle_mesh(stl_file)

    # 从网格采样点云
    point_cloud = mesh.sample_points_uniformly(number_of_points=num_points)

    return point_cloud


# 2. 加载 Realsense 相机的点云
def load_realsense_point_cloud(ply_file):
    # 读取相机拍摄的点云
    return o3d.io.read_point_cloud(ply_file)


# 3. 归一化点云：将点云的坐标缩放到一个统一的大小
def normalize_point_cloud(point_cloud):
    # 获取点云坐标
    points = np.asarray(point_cloud.points)

    # 计算点云的边界框
    min_coords = np.min(points, axis=0)
    max_coords = np.max(points, axis=0)

    # 计算边界框的尺度（最大值和最小值之间的差距）
    scale = np.max(max_coords - min_coords)

    # 归一化：将点云缩放到 [0, 1] 范围
    normalized_points = (points - min_coords) / scale

    # 更新点云的坐标
    point_cloud.points = o3d.utility.Vector3dVector(normalized_points)

    return point_cloud


# 设置文件路径
stl_file_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\ICP_Iterative_Closest_Point\box.stl"  # 长方体的 STL 文件路径
realsense_ply_file = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\ABB-11-6\Dataset\colored_point_cloud120302.ply"  # Realsense 点云文件路径

# 生成 STL 文件对应的点云
stl_point_cloud = stl_to_point_cloud(stl_file_path, 30000)

# 加载 Realsense 相机的点云
realsense_point_cloud = load_realsense_point_cloud(realsense_ply_file)

# 归一化两者的点云
stl_point_cloud_normalized = normalize_point_cloud(stl_point_cloud)
realsense_point_cloud_normalized = normalize_point_cloud(realsense_point_cloud)

# 可视化归一化后的 STL 点云和 Realsense 点云（查看大小是否一致）
o3d.visualization.draw_geometries([stl_point_cloud_normalized, realsense_point_cloud_normalized])

# 可选：保存归一化后的点云为 .ply 文件
o3d.io.write_point_cloud("Dataset_Point_Clouds/normalized_stl_point_cloud.ply", stl_point_cloud_normalized)
o3d.io.write_point_cloud("Dataset_Point_Clouds/normalized_realsense_point_cloud.ply", realsense_point_cloud_normalized)





'''
E:\ABB-Project\ABB_wrs\venv\Scripts\python.exe E:\ABB-Project\ABB_wrs\suyixuan\ABB\Task3_ICP_learning\ICP_registration2.py 
显示源点云...
显示目标点云...
Fitness: 1.0
RMSE: 0.04272162713150202
Transformation Matrix:

[[ 9.98695073e-01 -4.29472256e-03 -5.08892267e-02 -1.89501654e-02]
 [ 2.83316108e-04  9.96908356e-01 -7.85725674e-02 -2.95379411e-02]
 [ 5.10693429e-02  7.84556141e-02  9.95608678e-01 -1.87374895e-02]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]

Process finished with exit code 0


'''