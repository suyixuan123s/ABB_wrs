"""
Author: Yixuan Su
Date: 2024/11/17 10:26
File: read_triangle_mesh.py
"""

import os

import open3d as o3d

# 加载 STL 文件
file_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Task4_ICP_GOFA5\meshes\blood_tube5.STL"
mesh = o3d.io.read_triangle_mesh(file_path)

# 检查是否加载成功
if mesh.is_empty():
    print("加载 STL 文件失败，请检查路径或文件格式。")
else:
    # 为 mesh 添加颜色
    mesh.paint_uniform_color([0, 1, 0])  # RGB 格式

    # 显示 mesh
    o3d.visualization.draw_geometries([mesh], window_name="STL 文件显示 - 带颜色",
                                      width=1280, height=720, mesh_show_wireframe=True, mesh_show_back_face=False)