"""
Author: Yixuan Su
Date: 2024/12/03 20:38
File: convert_centimeters_to_meters.py
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


# 3. 将 STL 点云的单位从厘米转换为米
def convert_centimeters_to_meters(point_cloud):
    # 获取点云坐标
    points = np.asarray(point_cloud.points)

    # 将所有点坐标除以 100，将厘米转换为米
    points = points / 100

    # 更新点云的坐标
    point_cloud.points = o3d.utility.Vector3dVector(points)

    return point_cloud


# 设置文件路径
stl_file_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\ICP_Iterative_Closest_Point\box.stl"  # 长方体的 STL 文件路径

# 生成 STL 文件对应的点云
stl_point_cloud = stl_to_point_cloud(stl_file_path, 30000)

# 将 STL 点云从厘米转换为米
stl_point_cloud_in_meters = convert_centimeters_to_meters(stl_point_cloud)

# 可选：保存转换后的点云为 .ply 文件
o3d.io.write_point_cloud("Dataset_Point_Clouds/converted_stl_point_cloud.ply", stl_point_cloud_in_meters)

o3d.visualization.draw_geometries([stl_point_cloud_in_meters])
