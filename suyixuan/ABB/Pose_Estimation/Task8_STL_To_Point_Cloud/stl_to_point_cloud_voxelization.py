"""
Author: Yixuan Su
Date: 2024/12/16 14:21
File: stl_to_point_cloud_voxelization.py
Description: 在计算机图形学、三维建模和几何处理等领域，网格（Mesh）是用于表示三维物体的一种数学模型，
通常由一组顶点（点）、边和面组成。
网格是三维物体的离散化表示，它将物体的表面结构转化为有限数量的简单几何元素，通常是三角形或四边形。
"""
import os

import trimesh
import open3d as o3d
import numpy as np


def stl_to_point_cloud_voxelization(stl_path, voxel_size=0.1, save_path="stl_to_point_cloud_voxelization.ply"):

    # 确保保存路径的目录存在
    if not os.path.exists(os.path.dirname(save_path)):
        os.makedirs(os.path.dirname(save_path))

    # 加载STL模型
    mesh = trimesh.load_mesh(stl_path)

    # 体素化网格
    voxel_grid = mesh.voxelized(voxel_size)

    # 获取体素网格中的点
    points = voxel_grid.points

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
stl_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task3_ICP_pre-processing\temp.stl"
save_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task8_STL_To_Point_Cloud\Datasets\stl_to_point_cloud_voxelization1.ply"
point_cloud = stl_to_point_cloud_voxelization(stl_path, 0.0007, save_path)


