"""
Author: Yixuan Su
Date: 2024/11/19 20:40
File: Tiling_point_cloud_to_desktop.py
Description:
"""

import os
import numpy as np
import basis.robot_math as rm
import visualization.panda.world as wd
import modeling.geometric_model as gm
import open3d as o3d
import vision.depth_camera.pcd_data_adapter as vdda
from robot_sim.robots.GOFA55.GOFA5 import GOFA5  # 修改导入路径


def VisualizePCD(pcd_path):
    """
    加载点云文件，并将其转换为 NumPy 数组和颜色数据。

    参数:
        pcd_path (str): 点云文件路径，可以是 .ply, .pcd 等格式

    返回:
        pcd_np (numpy.ndarray): 点云的坐标数据
        pcd_colors (numpy.ndarray): 点云的颜色数据
    """
    # 使用 Open3D 读取点云文件
    pcd = o3d.io.read_point_cloud(pcd_path)

    # 将点云数据转换为 NumPy 数组（坐标部分）
    pcd_np = np.asarray(pcd.points)

    # 检查点云是否包含颜色信息
    if pcd.has_colors():
        # 如果有颜色信息，获取 RGB 数据，并转为 NumPy 数组
        pcd_colors = np.asarray(pcd.colors)
    else:
        # 如果没有颜色信息，默认使用红色
        pcd_colors = np.array([[1, 0, 0]] * len(pcd_np))  # 默认红色

    return pcd_np, pcd_colors


class GOFA5Demo:
    def __init__(self):
        # 初始化 Panda3D 世界环境
        self.base = wd.World(cam_pos=[0.63, 0.72, 1.26], lookat_pos=[0, 0, 0])
        gm.gen_frame().attach_to(self.base)
        # 添加世界坐标系原点
        gm.gen_frame(pos=[0, 0, 0], rotmat=np.eye(3), length=1, thickness=0.05).attach_to(self.base)

    def setup_robot(self):
        # 初始化 gofa5 机器人并添加到 Panda3D 场景中
        self.robot = GOFA5(enable_cc=True)
        self.robot.hnd.jaw_to(0.06)
        self.robot.gen_meshmodel(toggle_tcpcs=False, toggle_jntscs=False).attach_to(self.base)

    def add_point_cloud(self, pcd_path, camera_to_table_transform):
        # 使用 VisualizePCD 加载点云文件
        pcd_np, pcd_colors = VisualizePCD(pcd_path)

        # 如果需要添加 alpha 通道
        pcd_colors = np.concatenate((pcd_colors, np.ones((len(pcd_colors), 1))), axis=1)  # 添加 alpha 通道

        # 使用齐次变换矩阵将点云从相机坐标系转换到桌面坐标系
        pcd_homogeneous = np.concatenate((pcd_np, np.ones((pcd_np.shape[0], 1))), axis=1)  # 增加齐次坐标
        pcd_transformed = (camera_to_table_transform @ pcd_homogeneous.T).T  # 应用变换矩阵
        pcd_transformed = pcd_transformed[:, :3]  # 去掉齐次坐标

        # 保留点云的原始色彩信息并展示转换后的点云
        gm.gen_pointcloud(pcd_transformed, rgbas=pcd_colors, pntsize=3).attach_to(self.base)

    def run(self):
        # 运行可视化环境
        self.base.run()


if __name__ == '__main__':
    gofa_demo = GOFA5Demo()
    gofa_demo.setup_robot()

    camera_to_table_transform = np.array([[-0.99936647, -0.0332937, 0.01257708, 0.525],
                                          [-0.03489865, 0.84740238, -0.52980307, 0.76],
                                          [0.00698126, -0.52990635, -0.84802743, 1.25],
                                          [0., 0., 0., 1.]])
    # 添加桌面上的点云（替换为您的点云文件路径）
    gofa_demo.add_point_cloud(r'E:\ABB-Project\ABB_wrs\suyixuan\ABB\depth_anything_v2\Point_cloud_Dataset\color_image_20241211-100240.ply', camera_to_table_transform)
    gofa_demo.run()
