"""
Author: Yixuan Su
Date: 2024/11/19 20:40
File: Tiling_point_cloud_to_desktop1.py
Description:
"""

import numpy as np
import open3d as o3d
import basis.robot_math as rm  # 确保此模块包含了旋转矩阵的生成函数
import modeling.geometric_model as gm
import visualization.panda.world as wd
from robot_sim.robots.gofa5.gofa5 import GOFA5

# 手动调节的固定参数
alpha, beta, gamma = -154.0, -3.5, 0.5
tx, ty, tz = 0.47, -0.64, 1.41


def get_transformation_matrix(alpha_deg, beta_deg, gamma_deg, tx, ty, tz):
    """根据手调参数生成转换矩阵"""
    # 将角度转换为弧度
    alpha_rad = np.radians(alpha_deg)
    beta_rad = np.radians(beta_deg)
    gamma_rad = np.radians(gamma_deg)

    # 构建旋转矩阵
    rotation_matrix = (
        rm.rotmat_from_euler(alpha_rad, beta_rad, gamma_rad)
    )

    # 构造齐次变换矩阵
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = [tx, ty, tz]

    return transformation_matrix


def apply_transformation_to_point_cloud(pcd, transformation_matrix):
    """将点云从相机坐标系转换到仿真坐标系，并保留颜色信息"""
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors) if pcd.has_colors() else np.array([[1, 0, 0]] * len(points))  # 默认红色

    # 转换点云坐标
    ones = np.ones((points.shape[0], 1))
    homogeneous_points = np.hstack((points, ones))
    transformed_homogeneous_points = homogeneous_points @ transformation_matrix.T
    transformed_points = transformed_homogeneous_points[:, :3]

    return transformed_points, colors


def main():
    # 初始化 Panda3D 仿真环境
    base = wd.World(cam_pos=[1.5, 1.5, 1.5], lookat_pos=[0, 0, 0])

    # 加载并初始化 gofa5 机器人
    robot = GOFA5(enable_cc=True)
    robot.hnd.jaw_to(0.06)
    robot.gen_meshmodel(toggle_tcpcs=False, toggle_jntscs=False).attach_to(base)

    # 加载新的点云文件
    pcd_path = "E:\ABB-Project\ABB_wrs\suyixuan\ABB\depth_anything_v2\Point_cloud_Dataset\color_image_20241211-100240.ply"  # 替换为新的点云文件路径
    pcd = o3d.io.read_point_cloud(pcd_path)

    # 获取转换矩阵
    transformation_matrix = get_transformation_matrix(alpha, beta, gamma, tx, ty, tz)
    print("转换矩阵:\n", transformation_matrix)

    # 应用转换矩阵，将点云平铺到仿真桌面上
    transformed_points, colors = apply_transformation_to_point_cloud(pcd, transformation_matrix)

    # 显示转换后的点云并保留原始颜色
    desk_pcd = gm.gen_pointcloud(transformed_points, rgbas=np.hstack((colors, np.ones((colors.shape[0], 1)))),
                                 pntsize=3)
    desk_pcd.attach_to(base)

    # 运行 Panda3D 仿真环境
    base.run()


if __name__ == "__main__":
    main()
