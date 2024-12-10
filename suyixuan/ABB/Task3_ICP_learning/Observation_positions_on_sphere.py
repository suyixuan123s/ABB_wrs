"""
Author: Yixuan Su
Date: 2024/11/26
File: observation_positions_on_sphere.py
Description: Generate observation positions on a sphere and visualize them with a point cloud representation, including the center point from camera.
"""
import numpy as np
import open3d as o3d
from math import pi, sin, cos


# 生成球体上的观测位置
def generate_observation_positions(num_positions=20, radius=5):
    """
    在一个球体表面生成均匀分布的观测点，每个观测点模拟一个相机位置。
    :param num_positions: 生成的观测点数量
    :param radius: 球体的半径
    :return: 观测位置列表
    """
    observation_positions = []
    for i in range(num_positions):
        theta = np.arccos(1 - 2 * (i + 0.5) / num_positions)  # 均匀分布角度
        phi = 2 * pi * ((i + 0.5) / num_positions)  # 均匀分布角度
        x = radius * sin(theta) * cos(phi)
        y = radius * sin(theta) * sin(phi)
        z = radius * cos(theta)
        observation_positions.append([x, y, z])
    return np.array(observation_positions)


# 可视化球体上的观测点以及相机点云中心
def visualize_observation_positions(observation_positions, camera_center):
    """
    将观测位置和相机点云的中心显示为一个点云。
    :param observation_positions: 观测点位置列表
    :param camera_center: 相机点云的中心位置
    """
    # 创建观测点的点云对象
    observation_pcd = o3d.geometry.PointCloud()
    observation_pcd.points = o3d.utility.Vector3dVector(observation_positions)
    observation_pcd.paint_uniform_color([1, 0, 0])  # 红色显示观测点

    # 创建相机点云中心的点云对象
    camera_pcd = o3d.geometry.PointCloud()
    camera_pcd.points = o3d.utility.Vector3dVector(np.array([camera_center]))
    camera_pcd.paint_uniform_color([0, 1, 0])  # 绿色显示相机点云中心

    # 可视化点云
    o3d.visualization.draw_geometries([observation_pcd, camera_pcd])


# 主程序
if __name__ == "__main__":
    # 已知的相机点云中心位置
    camera_center = [-0.06346996, -0.07218969, 1.4510558]

    # 根据相机点云中心位置相对于相机原点计算球体半径
    radius = np.linalg.norm(np.array(camera_center))
    print("The radius is:", radius)

# 生成观测位置
observation_positions = generate_observation_positions(num_positions=100, radius=radius)

# 可视化观测位置和相机点云中心
visualize_observation_positions(observation_positions, camera_center)
"""
This script generates observation positions uniformly distributed over a sphere's surface, then visualizes them along with the camera's point cloud center as a point cloud using Open3D.
You can adjust the number of observation positions and the radius of the sphere as needed.
"""
