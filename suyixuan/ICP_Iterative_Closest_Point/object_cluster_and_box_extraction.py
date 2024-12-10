"""
Author: Yixuan Su
Date: 2024/12/03 10:37
File: object_cluster_and_box_extraction.py
Description: 
"""

import open3d as o3d
import numpy as np
from matplotlib import pyplot as plt

# 1. 加载点云文件
camera_pcd_path = "cropped_point_cloud1203.ply"  # 替换为您的点云文件路径
camera_pcd = o3d.io.read_point_cloud(camera_pcd_path)

# 可视化原始点云
o3d.visualization.draw_geometries([camera_pcd], window_name="Original Point Cloud")

# 2. 平面分割：移除桌面或地面
plane_model, inliers = camera_pcd.segment_plane(
    distance_threshold=0.04,  # 平面距离阈值
    ransac_n=3,  # 每次拟合平面时使用的最小点数
    num_iterations=1000  # RANSAC的最大迭代次数
)

# 打印出平面方程的系数 a, b, c, d
a, b, c, d = plane_model
print(f"平面方程的系数: a = {a}, b = {b}, c = {c}, d = {d}")

# 提取平面以外的点云（即物体点云）
objects_pcd = camera_pcd.select_by_index(inliers, invert=True)

# 可视化物体点云
objects_pcd.paint_uniform_color([0, 1, 0])  # 绿色表示物体点云
o3d.visualization.draw_geometries([objects_pcd], window_name="Objects Point Cloud")

# 3. 聚类：提取独立物体簇
# 使用 DBSCAN 聚类
labels = np.array(objects_pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))
max_label = labels.max()
print(f"Detected {max_label + 1} clusters")

# 提取簇标签为 0 的点云（例如提取第一个簇）
cluster_0_indices = np.where(labels == 0)[0]
cluster_0_pcd = objects_pcd.select_by_index(cluster_0_indices)

# 可视化物体点云
objects_pcd.paint_uniform_color([0, 0.5, 0])  # 绿色表示物体点云
o3d.visualization.draw_geometries([cluster_0_pcd], window_name="cluster_0_pcd Point Cloud")

# 统计每个簇的点数
unique, counts = np.unique(labels, return_counts=True)
print(dict(zip(unique, counts)))

# 可视化聚类结果
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))[:, :3]  # 颜色映射
colors[labels < 0] = 0  # 未分类点为黑色
objects_pcd.colors = o3d.utility.Vector3dVector(colors)
o3d.visualization.draw_geometries([objects_pcd], window_name="Clustered Point Cloud")

# 4. 提取最大的点云簇（假设盒子是最大簇）
cluster_sizes = [np.sum(labels == i) for i in range(max_label + 1)]
box_label = np.argmax(cluster_sizes)  # 最大簇的索引
box_pcd = objects_pcd.select_by_index(np.where(labels == box_label)[0])

# 可视化提取的盒子点云
box_pcd.paint_uniform_color([1, 0, 0])  # 红色表示盒子
o3d.visualization.draw_geometries([box_pcd], window_name="Extracted Box Point Cloud")

# 5. 保存分割后的盒子点云
o3d.io.write_point_cloud("extracted_box.ply", box_pcd)
print("Box point cloud saved as 'extracted_box.ply'")
