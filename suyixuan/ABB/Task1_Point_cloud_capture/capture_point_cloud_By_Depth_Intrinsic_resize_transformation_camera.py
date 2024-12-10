"""
Author: Yixuan Su
Date: 2024/11/18 13:56
File: capture_point_cloud_By_Depth_Intrinsic_simulation_resize.py
"""

import numpy as np
import open3d as o3d


def compute_transformation(alpha_deg, beta_deg, gamma_deg, tx, ty, tz):
    # 将角度转换为弧度
    alpha = np.radians(alpha_deg)
    beta = np.radians(beta_deg)
    gamma = np.radians(gamma_deg)

    # 构造旋转矩阵 (X, Y, Z 轴)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(alpha), -np.sin(alpha)],
        [0, np.sin(alpha), np.cos(alpha)]
    ])

    Ry = np.array([
        [np.cos(beta), 0, np.sin(beta)],
        [0, 1, 0],
        [-np.sin(beta), 0, np.cos(beta)]
    ])

    Rz = np.array([
        [np.cos(gamma), -np.sin(gamma), 0],
        [np.sin(gamma), np.cos(gamma), 0],
        [0, 0, 1]
    ])

    # 组合旋转矩阵
    rotation_matrix = Rz @ Ry @ Rx

    # 构造齐次变换矩阵 (相机到仿真坐标系的外参)
    transformation_camera_to_sim = np.eye(4)
    transformation_camera_to_sim[:3, :3] = rotation_matrix
    transformation_camera_to_sim[:3, 3] = [tx, ty, tz]

    # 计算变换矩阵的逆矩阵
    transformation_sim_to_camera = np.linalg.inv(transformation_camera_to_sim)

    return transformation_camera_to_sim, transformation_sim_to_camera


def transform_point_cloud(point_cloud, transformation_matrix):
    """
    使用变换矩阵将点云转换到仿真坐标系。
    """
    points = np.asarray(point_cloud.points)  # 获取点云的所有点
    # 在点的坐标上应用变换矩阵（齐次坐标）
    ones = np.ones((points.shape[0], 1))  # 添加一个列向量1，用于齐次坐标变换
    points_homogeneous = np.hstack([points, ones])  # 合并坐标和1，形成齐次坐标

    # 应用变换矩阵
    transformed_points = points_homogeneous.dot(transformation_matrix.T)[:, :3]  # 去掉最后一列
    transformed_point_cloud = point_cloud
    transformed_point_cloud.points = o3d.utility.Vector3dVector(transformed_points)  # 更新点云坐标

    return transformed_point_cloud


def load_point_cloud(file_path):
    """
    加载保存的点云文件 (.ply 格式)
    """
    point_cloud = o3d.io.read_point_cloud(file_path)
    return point_cloud


def main():
    # 加载已有的点云文件 (例如从 capture_point_cloud_By_Depth_Intrinsic_simulation_resize.py 中提取)
    input_point_cloud_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\ABB-11-6\Dataset\colored_point_cloud120302.ply"  # 已保存的点云文件路径
    point_cloud = load_point_cloud(input_point_cloud_path)

    # 设定旋转和平移参数
    alpha, beta, gamma = 27.5, -180.0, 180.0  # 旋转角度 (单位：度)
    tx, ty, tz = 0.42, -0.77, 1.23  # 平移 (单位：米)

    transformation_camera_to_sim, transformation_sim_to_camera = compute_transformation(alpha, beta, gamma, tx, ty, tz)

    # 使用变换矩阵将点云从相机坐标系转换到仿真坐标系
    reverted_point_cloud = transform_point_cloud(point_cloud, transformation_sim_to_camera)

    # 保存转换后的点云
    save_path = "reverted_point_cloud.ply"  # 结果保存的文件路径
    o3d.io.write_point_cloud(save_path, reverted_point_cloud)
    print(f"转换后的点云已保存为 {save_path}")

    # 可视化还原后的点云
    o3d.visualization.draw_geometries([reverted_point_cloud])


if __name__ == "__main__":
    main()
