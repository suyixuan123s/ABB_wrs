"""
Author: Yixuan Su
Date: 2024/12/16 14:20
File: stl_to_point_cloud_mesh_simplification_and_reconstruction.py
Description: Load STL file, simplify mesh, reconstruct surface using Open3D, and save point cloud.
"""


import os

import trimesh
import open3d as o3d
import numpy as np


def stl_to_point_cloud_mesh_simplification_and_reconstruction(stl_path, num_points=1000,
                                                              save_path="point_cloud_simplification.ply"):
    # 确保保存路径的目录存在
    if not os.path.exists(os.path.dirname(save_path)):
        os.makedirs(os.path.dirname(save_path))

    # 加载STL模型
    mesh = trimesh.load_mesh(stl_path)

    # 网格简化
    print("Simplifying mesh...")
    # simplify_quadratic_decimation 方法使用二次简化算法将原始网格简化为 num_points 个点。
    mesh_simplified = mesh.simplify_quadric_decimation(num_points)

    # 转换为Open3D网格对象
    mesh_o3d = o3d.geometry.TriangleMesh()
    mesh_o3d.vertices = o3d.utility.Vector3dVector(mesh_simplified.vertices)
    mesh_o3d.triangles = o3d.utility.Vector3iVector(mesh_simplified.faces)
    mesh_o3d.compute_vertex_normals()  # 计算法线

    # 曲面重建（Poisson重建）
    print("Performing Poisson surface reconstruction...")
    # create_from_point_cloud_poisson 是一个泊松表面重建方法。
    # 它根据输入的点云生成一个三角形网格。
    # depth=9 控制重建的精度和细节程度。较高的深度会产生更精细的网格。
    poisson_mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        mesh_o3d.sample_points_uniformly(num_points), depth=9)

    # 从重建的曲面网格中采样点云
    print("Sampling points from the reconstructed mesh...")
    point_cloud = poisson_mesh.sample_points_uniformly(num_points)

    # 可视化点云
    print("Visualizing point cloud...")
    o3d.visualization.draw_geometries([point_cloud])

    # 保存点云
    print(f"Saving point cloud to {save_path}")
    o3d.io.write_point_cloud(save_path, point_cloud)

    return point_cloud


# 使用方法
if __name__ == "__main__":
    stl_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task8_STL_To_Point_Cloud\temp.stl"
    save_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task8_STL_To_Point_Cloud\Datasets\stl_to_point_cloud_mesh_simplification_and_reconstruction.ply"
    point_cloud = stl_to_point_cloud_mesh_simplification_and_reconstruction(stl_path, 2000, save_path)
