"""
Author: Yixuan Su
Date: 2024/12/16 14:20
File: stl_to_point_cloud_mesh_simplification_and_reconstruction.py
Description: 
"""
import trimesh
import open3d as o3d
import numpy as np


def stl_to_point_cloud_mesh_simplification_and_reconstruction(stl_path, num_points=1000, save_path="point_cloud_simplification.ply"):
    # 加载STL模型
    mesh = trimesh.load_mesh(stl_path)

    # 网格简化
    mesh_simplified = mesh.simplify_quadratic_decimation(num_points)

    # 曲面重建（使用Poisson重建）
    poisson_mesh = mesh_simplified.poisson_surface_reconstruction()

    # 从简化网格中采样点
    points = poisson_mesh.sample(num_points)

    # 转换为Open3D点云对象
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    # 可视化点云
    o3d.visualization.draw_geometries([point_cloud])

    # 保存点云
    o3d.io.write_point_cloud(save_path, point_cloud)
    print(f"Point cloud saved to {save_path}")

    return point_cloud


# 使用方法
stl_path = r"/suyixuan/ABB/Pose_Estimation/Task3_ICP_pre-processing/temp.stl"
save_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task7\Task8_stl_to_point_cloud\Datasets\stl_to_point_cloud_mesh_simplification_and_reconstruction.ply"
point_cloud = stl_to_point_cloud_mesh_simplification_and_reconstruction(stl_path, 2000, save_path)
