"""
Author: Yixuan Su
Date: 2024/11/17 10:26
File: single_stl_to_point_cloud.py
"""

import open3d as o3d
import numpy as np


class VisualizePCD:
    def __init__(self, pcd_path):
        """
        初始化 VisualizePCD 类，加载点云文件。

        参数:
            pcd_path (str): 点云文件路径，可以是 .ply, .pcd 等格式
        """
        # 使用 Open3D 读取点云文件
        self.pcd = o3d.io.read_point_cloud(pcd_path)

        # 将点云数据转换为 NumPy 数组
        self.pcd_np = np.asarray(self.pcd.points)

        # 检查点云是否包含颜色信息
        if self.pcd.has_colors():
            # 如果有颜色信息，获取 RGB 数据，并转为 NumPy 数组
            self.pcd_colors = np.asarray(self.pcd.colors)
        else:
            # 如果没有颜色信息，默认使用红色
            self.pcd_colors = np.array([[1, 0, 0]] * len(self.pcd_np))

    def show(self):
        """
        使用 Open3D 显示点云。
        """
        o3d.visualization.draw_geometries([self.pcd])

    def get_pcd(self):
        """
        返回点云对象。

        返回:
            open3d.geometry.PointCloud: 点云对象
        """
        return self.pcd

    def get_pcd_np(self):
        """
        返回点云的 NumPy 数组（坐标部分）。

        返回:
            numpy.ndarray: 点云的坐标数据
        """
        return self.pcd_np

    def get_colors(self):
        """
        返回点云的颜色数据（RGB 格式）。

        返回:
            numpy.ndarray: 点云的颜色数据
        """
        return self.pcd_colors
