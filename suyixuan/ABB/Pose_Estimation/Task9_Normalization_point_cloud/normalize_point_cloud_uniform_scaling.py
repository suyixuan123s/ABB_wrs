"""
Author: Yixuan Su
Date: 2024/12/16 19:06
File: normalize_point_cloud_uniform_scaling.py
Description: 
"""


import open3d as o3d
import numpy as np

def normalize_point_cloud_uniform_scaling(point_cloud):
    """
    将点云进行等比例缩放，保持原始形状和比例，并缩放到 [0, 1] 范围。
    """
    # 获取点云坐标
    points = np.asarray(point_cloud.points)

    # 计算点云的边界框的最小值和最大值
    min_coords = np.min(points, axis=0)
    max_coords = np.max(points, axis=0)

    # 计算边界框的整体尺寸
    bbox_size = max_coords - min_coords

    # 计算最大边长（作为等比例缩放的尺度）
    max_extent = np.max(bbox_size)

    # 将点云等比例缩放到 [0, 1] 范围
    normalized_points = (points - min_coords) / max_extent

    # 更新点云的坐标
    point_cloud.points = o3d.utility.Vector3dVector(normalized_points)

    return point_cloud


# 加载点云
input_file = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task9_Normalization_point_cloud\source.ply"
output_file = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task9_Normalization_point_cloud\source_normalized_uniform.ply"

# 读取点云
point_cloud = o3d.io.read_point_cloud(input_file)

# 可视化归一化前的点云
print("归一化前的点云：")
o3d.visualization.draw_geometries([point_cloud])

# 等比例缩放点云
normalized_point_cloud = normalize_point_cloud_uniform_scaling(point_cloud)

# 可视化归一化后的点云
print("归一化后的点云（等比例缩放）：")
o3d.visualization.draw_geometries([normalized_point_cloud])

# 保存归一化后的点云
o3d.io.write_point_cloud(output_file, normalized_point_cloud)
print(f"归一化后的点云已保存至: {output_file}")
