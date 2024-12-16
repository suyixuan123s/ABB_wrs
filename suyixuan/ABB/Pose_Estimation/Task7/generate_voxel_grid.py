"""
Author: Yixuan Su
Date: 2024/12/15 18:18
File: generate_voxel_grid.py
Description: 
"""
import numpy as np
import open3d as o3d

# 读取STL文件
mesh = o3d.io.read_triangle_mesh(r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task3_ICP_pre-processing\temp.stl")
bbox = mesh.get_axis_aligned_bounding_box()
min_bound = np.asarray(bbox.min_bound)
max_bound = np.asarray(bbox.max_bound)

# 设置体素大小
voxel_size = 0.001  # 可以根据需要调整体素大小
# 计算每个小正方体的顶点
def generate_voxel_grid(min_bound, max_bound, voxel_size):
    x_vals = np.arange(min_bound[0], max_bound[0], voxel_size)
    y_vals = np.arange(min_bound[1], max_bound[1], voxel_size)
    z_vals = np.arange(min_bound[2], max_bound[2], voxel_size)

    # 生成每个体素的8个顶点
    voxel_points = []
    for x in x_vals:
        for y in y_vals:
            for z in z_vals:
                # 顶点坐标，表示体素的一个角点
                voxel_points.append([x, y, z])

    return np.array(voxel_points)


# 获取体素网格的顶点
voxel_points = generate_voxel_grid(min_bound, max_bound, voxel_size)

# 将这些点云转换为Open3D点云
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(voxel_points)

# 显示点云
o3d.visualization.draw_geometries([pcd], window_name="Voxelized Point Cloud")

# 保存点云
o3d.io.write_point_cloud("voxelized_point_cloud.ply", pcd)
