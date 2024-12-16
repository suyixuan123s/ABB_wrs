"""
Author: Yixuan Su
Date: 2024/12/16 14:23
File: stl_to_point_cloud_from_vertices.py
Description: 
"""

import trimesh
import open3d as o3d


def stl_to_point_cloud_from_vertices(stl_path, save_path="point_cloud_vertices.ply"):
    # 加载STL模型
    mesh = trimesh.load_mesh(stl_path)

    # 获取模型的顶点
    vertices = mesh.vertices

    # 转换为Open3D点云对象
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(vertices)

    # 可视化点云
    o3d.visualization.draw_geometries([point_cloud])

    # 保存点云
    o3d.io.write_point_cloud(save_path, point_cloud)
    print(f"Point cloud saved to {save_path}")

    return point_cloud


# 使用方法
stl_path = r"/suyixuan/ABB/Pose_Estimation/Task3_ICP_pre-processing/temp.stl"
save_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task7\Task8_stl_to_point_cloud\Datasets\stl_to_point_cloud_from_vertices.ply"
point_cloud = stl_to_point_cloud_from_vertices(stl_path, save_path)
