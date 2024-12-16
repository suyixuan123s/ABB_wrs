"""
Author: Yixuan Su
Date: 2024/12/16 14:36
File: normalize_point_cloud.py
Description: 
"""

import open3d as o3d
import numpy as np

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

# 加载点云文件
input_file = r'/suyixuan/ABB/Pose_Estimation/Task8_STL_To_Point_Cloud/Datasets/stl_to_point_cloud_voxelization1.ply'
point_cloud = o3d.io.read_point_cloud(input_file)

# 可视化归一化前的点云
o3d.visualization.draw_geometries([point_cloud], window_name="Source Point Cloud")

# 对点云进行归一化处理
normalized_point_cloud = normalize_point_cloud(point_cloud)

# 可视化归一化后的点云
o3d.visualization.draw_geometries([normalized_point_cloud], window_name="Normalized Point Cloud")

# 保存归一化后的点云
output_file = r'/suyixuan/ABB/Pose_Estimation/Task8_STL_To_Point_Cloud/Datasets/stl_to_point_cloud_voxelization_normalized1.ply'
o3d.io.write_point_cloud(output_file, normalized_point_cloud)

print(f"归一化后的点云已保存至: {output_file}")
