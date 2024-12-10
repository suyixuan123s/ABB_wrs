"""
Author: Yixuan Su
Date: 2024/12/03 12:00
File: pcd_voxel_down_sample.py
Description: 
"""

import open3d as o3d

# 读取点云
pcd = o3d.io.read_point_cloud(r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\ABB-11-6\Dataset\colored_point_cloud1203.ply")

# 进行体素网格降采样
voxel_size = 0.5  # 设置体素大小
pcd_voxel = pcd.voxel_down_sample(voxel_size)

# 可视化裁剪后的点云
o3d.visualization.draw_geometries([pcd_voxel])
