"""
Author: Yixuan Su
Date: 2024/11/19 20:40
File: Tiling_point_cloud_to_desktop_resize.py
Description:
"""

import os
import numpy as np
import open3d as o3d
import visualization.panda.world as wd
import modeling.geometric_model as gm
from robot_sim.robots.gofa5.gofa5 import GOFA5  # 导入 Task4_ICP_GOFA5.py 中的机器人模型

# 仿真平台桌面范围（单位：米）
x_range = (-0.14, 1.06)  # x 轴范围
y_range = (-0.71, 0.80)  # y 轴范围
z_range = (-0.015, 0.60)  # z 轴范围

# 已知的手调参数
alpha, beta, gamma = -148.0, -0.4, -178.0
tx, ty, tz = 0.525, 0.76, 1.25


def get_transformation_matrix(alpha_deg, beta_deg, gamma_deg, tx, ty, tz):
    alpha = np.radians(alpha_deg)
    beta = np.radians(beta_deg)
    gamma = np.radians(gamma_deg)

    # 构建旋转矩阵
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

    # 构造齐次变换矩阵
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = [tx, ty, tz]

    return transformation_matrix


def transform_points(points, transformation_matrix):
    """将点云从相机坐标系转换到仿真坐标系"""
    ones = np.ones((points.shape[0], 1))
    homogeneous_points = np.hstack((points, ones))
    transformed_homogeneous_points = homogeneous_points @ transformation_matrix.T
    transformed_points = transformed_homogeneous_points[:, :3]
    return transformed_points


def filter_points_on_desk(points, colors, x_range, y_range, z_range):
    """提取在桌面区域的点和对应的颜色"""
    mask = (
            (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1]) &
            (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1]) &
            (points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])
    )
    return points[mask], colors[mask]


def main():
    # 初始化 Panda3D 仿真环境
    base = wd.World(cam_pos=[1.5, 1.5, 1.5], lookat_pos=[0, 0, 0])

    # 加载并初始化 gofa5 机器人和桌子
    robot = GOFA5(enable_cc=True)
    robot.hnd.jaw_to(0.06)
    robot.gen_meshmodel(toggle_tcpcs=False, toggle_jntscs=False).attach_to(base)

    # 添加仿真坐标系的中心和 XYZ 轴信息
    gm.gen_frame(pos=[0, 0, 0], rotmat=np.eye(3), length=1, thickness=0.05).attach_to(base)

    # 加载点云
    pcd_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB_Point_Cloud\colored_point_cloud1118.ply"  # 请更换为实际路径
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors) if pcd.has_colors() else np.array([[1, 0, 0]] * len(points))  # 默认红色

    # 获取转换矩阵
    transformation_matrix = get_transformation_matrix(alpha, beta, gamma, tx, ty, tz)

    # 转换到仿真坐标系
    transformed_points = transform_points(points, transformation_matrix)

    # 提取桌面上的点和颜色
    desk_points, desk_colors = filter_points_on_desk(transformed_points, colors, x_range, y_range, z_range)

    # 创建桌面点云对象，并显示在 Panda3D 中
    gm.gen_pointcloud(desk_points, rgbas=np.hstack((desk_colors, np.ones((desk_colors.shape[0], 1)))),
                      pntsize=3).attach_to(base)

    # 保存裁剪后的点云
    desk_pcd = o3d.geometry.PointCloud()
    desk_pcd.points = o3d.utility.Vector3dVector(desk_points)
    desk_pcd.colors = o3d.utility.Vector3dVector(desk_colors)

    output_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB_Point_Cloud\cropped_point_cloud1.ply"
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    o3d.io.write_point_cloud(output_path, desk_pcd)
    print(f"裁剪后的点云已保存到: {output_path}")

    # 运行仿真环境
    base.run()


if __name__ == "__main__":
    main()
