"""
Author: Yixuan Su
Date: 2024/11/26 19:57
File: point_cloud_from_stl_and_camera.py
Description: Modified version to achieve uniform point cloud sampling from STL file.
"""
import numpy as np
import open3d as o3d
import pyvista as pv
import random
from math import pi, sin, cos


# 加载STL文件并生成点云
def load_stl_to_point_cloud(stl_file):
    mesh = o3d.io.read_triangle_mesh(stl_file)
    # 使用 Open3D 的均匀采样方法从 STL 网格生成点云
    point_cloud = mesh.sample_points_uniformly(number_of_points=5000)
    return point_cloud


# 将STL模型转换为点云并加入随机噪声以模拟真实世界的点云
def stl_to_point_cloud_with_noise(point_cloud, noise_factor=0.005):
    points = np.asarray(point_cloud.points)
    noisy_points = points + np.random.normal(scale=noise_factor, size=points.shape)
    noisy_point_cloud = o3d.geometry.PointCloud()
    noisy_point_cloud.points = o3d.utility.Vector3dVector(noisy_points)
    return noisy_point_cloud


# 从不同视角生成点云
def generate_point_cloud_from_view(stl_points, view_position, view_target, up_vector=np.array([0, 0, 1])):
    """
    给定视角（位置、目标、向上方向），生成相机视角下的点云。
    这可以通过视点投影的方式进行，模拟从该视角采集点云。
    """
    # 视角变换矩阵，假设相机正朝向目标
    direction = view_target - view_position
    direction /= np.linalg.norm(direction)  # 单位化方向向量

    # 模拟从该视角采集的点云（在实际应用中可能用其他方法来得到视角投影）
    projected_points = []
    for point in np.asarray(stl_points.points):
        # 计算相对视角中的点坐标
        offset = point - view_position
        dot_product = np.dot(offset, direction)
        if dot_product > 0:  # 仅采集前方的点云
            projected_points.append(point)

    projected_point_cloud = o3d.geometry.PointCloud()
    projected_point_cloud.points = o3d.utility.Vector3dVector(np.array(projected_points))
    return projected_point_cloud


# 模拟多个视角并生成点云
def generate_multiple_views_point_cloud(stl_points, num_views=10):
    views = []
    for i in range(num_views):
        # 随机选择一个视角（模拟一个球面上的点作为相机位置）
        theta = random.uniform(0, pi)  # 方位角
        phi = random.uniform(0, 2 * pi)  # 极角
        radius = 5  # 假设相机与物体的距离为5
        view_position = np.array([
            radius * sin(theta) * cos(phi),
            radius * sin(theta) * sin(phi),
            radius * cos(theta)
        ])
        view_target = np.array([0, 0, 0])  # 假设相机始终指向原点
        projected_points = generate_point_cloud_from_view(stl_points, view_position, view_target)
        views.append(projected_points)

    return views


# 可视化点云（用于调试）
def visualize_point_clouds(point_clouds):
    # 可视化多个点云
    o3d.visualization.draw_geometries(point_clouds)


# 点云配准（使用ICP算法）
def point_cloud_registration(source_cloud, target_cloud):
    # 配准方法
    threshold = 0.02  # 配准的阈值
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_cloud, target_cloud, threshold,
        np.eye(4), o3d.pipelines.registration.TransformationEstimationPointToPoint())

    # 使用 fitness 和 inlier_rmse 来了解配准结果
    print("ICP_Iterative_Closest_Point Converged:", reg_p2p.fitness > 0.5)  # 如果拟合度大于0.5，表示配准有效
    print("Fitness (匹配度): ", reg_p2p.fitness)
    print("Inlier RMSE (内点均方根误差): ", reg_p2p.inlier_rmse)
    print("Transformation Matrix (变换矩阵):\n", reg_p2p.transformation)
    return reg_p2p.transformation


# 主程序
if __name__ == "__main__":
    # STL文件路径
    stl_file = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Task4_ICP_GOFA5\meshes\rack_5ml_green.STL"  # 这里替换为你的STL文件路径

    # 加载STL文件并生成点云
    stl_points = load_stl_to_point_cloud(stl_file)

    # 为点云添加噪声
    noisy_stl_points = stl_to_point_cloud_with_noise(stl_points)

    # 给点云涂上绿色
    noisy_stl_points.paint_uniform_color([0, 1, 0])  # 绿色

    # 可视化点云
    o3d.visualization.draw_geometries([noisy_stl_points])

    # 生成多个视角下的点云
    generated_point_clouds = generate_multiple_views_point_cloud(stl_points, num_views=5)

    # 可视化生成的多个视角点云
    visualize_point_clouds(generated_point_clouds)

    # 假设相机拍摄的点云（这里可以替换为实际相机采集的点云）
    # 对于示例，我们将随机选择一个视角点云作为相机拍摄的点云
    camera_view_point_cloud = generated_point_clouds[0]  # 假设从第一个视角得到的点云

    # 点云配准，得到相机视角的位姿
    transformation = point_cloud_registration(camera_view_point_cloud, noisy_stl_points)

    print("Estimated Transformation Matrix:\n", transformation)
"""
This modified version ensures that the STL file is uniformly sampled to generate point clouds and improve visualization.
The functions have been streamlined to work directly with Open3D objects for more efficient operations.
"""
