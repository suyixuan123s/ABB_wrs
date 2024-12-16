"""
Author: Yixuan Su
Date: 2024/12/15 17:25
File: np_linalg_norm.py
Description: 
"""


import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt


def compute_point_distances(point_cloud):
    # 获取点云坐标
    points = np.asarray(point_cloud.points)

    # 计算点与点之间的欧几里得距离
    distances = []
    num_points = points.shape[0]

    for i in range(num_points - 1):
        dist = np.linalg.norm(points[i] - points[i + 1])
        distances.append(dist)

    return distances


def plot_distance_histogram(distances):
    # 绘制点间距离的直方图
    plt.hist(distances, bins=50, edgecolor='black')
    plt.title("Histogram of Point Distances")
    plt.xlabel("Distance")
    plt.ylabel("Frequency")
    plt.show()


# 读取点云文件（假设你已经有一个点云对象）
point_cloud = o3d.io.read_point_cloud(r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task3_ICP_pre-processing\TexturedPointCloud.ply")

# 计算点云中相邻点之间的距离
distances = compute_point_distances(point_cloud)

# 绘制距离的直方图
plot_distance_histogram(distances)

# 打印距离的统计信息
print(f"平均距离: {np.mean(distances)}")
print(f"最小距离: {np.min(distances)}")
print(f"最大距离: {np.max(distances)}")
