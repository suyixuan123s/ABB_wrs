"""
Author: Yixuan Su
Date: 2024/12/16 18:49
File: normalize_point_cloud_independently.py
Description: 
"""


import open3d as o3d
import numpy as np


def normalize_point_cloud_independently(point_cloud):
    """
    对点云的每个轴进行独立归一化到 [0, 1] 范围
    """
    points = np.asarray(point_cloud.points)
    min_coords = np.min(points, axis=0)
    max_coords = np.max(points, axis=0)
    scale = max_coords - min_coords  # 每个轴的尺度

    # 避免除以0
    scale[scale == 0] = 1.0

    # 逐轴归一化
    normalized_points = (points - min_coords) / scale
    point_cloud.points = o3d.utility.Vector3dVector(normalized_points)
    return point_cloud


def check_normalization(point_cloud, name="Point Cloud"):
    points = np.asarray(point_cloud.points)
    bbox_min = np.min(points, axis=0)
    bbox_max = np.max(points, axis=0)
    print(f"--- {name} ---")
    print(f"Bounding Box Min: {bbox_min}")
    print(f"Bounding Box Max: {bbox_max}")
    print(f"Range in X: {bbox_max[0] - bbox_min[0]:.6f}")
    print(f"Range in Y: {bbox_max[1] - bbox_min[1]:.6f}")
    print(f"Range in Z: {bbox_max[2] - bbox_min[2]:.6f}\n")


# 加载点云
source_ply_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task6_Point_cloud_processing\Dataset_Point_Cloud\normalized_realsense_point_cloud.ply"  # 修改为实际路径
target_ply_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task8_STL_To_Point_Cloud\Datasets\stl_to_point_cloud_voxelization_normalized.ply"  # 修改为实际路径

normalized_source_save_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task9_Normalization_point_cloud\normalize_point_cloud_independently\normalized_source_point_cloud.ply"
normalized_target_save_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task9_Normalization_point_cloud\normalize_point_cloud_independently\normalized_target_point_cloud.ply"


source = o3d.io.read_point_cloud(source_ply_path)
target = o3d.io.read_point_cloud(target_ply_path)

# 逐轴归一化
source = normalize_point_cloud_independently(source)
target = normalize_point_cloud_independently(target)



# 检查归一化效果
check_normalization(source, name="Source Point Cloud")
check_normalization(target, name="Target Point Cloud")



# 可视化
o3d.visualization.draw_geometries([source], window_name="Normalized Source Point Cloud")
o3d.visualization.draw_geometries([target], window_name="Normalized Target Point Cloud")


# 保存归一化后的点云
o3d.io.write_point_cloud(normalized_source_save_path, source)
o3d.io.write_point_cloud(normalized_target_save_path, target)

print(f"归一化后的源点云已保存至: {normalized_source_save_path}")
print(f"归一化后的目标点云已保存至: {normalized_target_save_path}")



"""

E:\ABB-Project\ABB_wrs\venv\Scripts\python.exe E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task9_Normalization_point_cloud\normalize_point_cloud_independently.py 
--- Source Point Cloud ---
Bounding Box Min: [0. 0. 0.]
Bounding Box Max: [1. 1. 1.]
Range in X: 1.000000
Range in Y: 1.000000
Range in Z: 1.000000

--- Target Point Cloud ---
Bounding Box Min: [0. 0. 0.]
Bounding Box Max: [1. 1. 1.]
Range in X: 1.000000
Range in Y: 1.000000
Range in Z: 1.000000

归一化后的源点云已保存至: E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task9_Normalization_point_cloud\normalize_point_cloud_independently\normalized_source_point_cloud.ply
归一化后的目标点云已保存至: E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task9_Normalization_point_cloud\normalize_point_cloud_independently\normalized_target_point_cloud.ply

Process finished with exit code 0




"""
