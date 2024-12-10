"""
Author: Yixuan Su
Date: 2024/11/17 10:26
File: all_stl_to_point_cloud_dataset.py
"""

import os
import open3d as o3d

# 定义STL文件所在的目录路径
stl_directory_path = r"/suyixuan/ABB/Task4_ICP_GOFA5/meshes"
# 定义点云文件保存的目录路径
point_cloud_save_directory = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Task4_ICP_GOFA5\stl_to_point_cloud_dataset"

# 创建保存点云的目录（如果不存在）
os.makedirs(point_cloud_save_directory, exist_ok=True)

# 遍历STL文件夹中的所有文件
for filename in os.listdir(stl_directory_path):
    if filename.endswith(".STL"):
        # 获取STL文件的完整路径
        stl_file_path = os.path.join(stl_directory_path, filename)

        # 读取STL文件
        mesh = o3d.io.read_triangle_mesh(stl_file_path)
        if not mesh:
            print(f"无法加载STL文件：{stl_file_path}")
            continue

        # 将三角网格转化为点云
        point_cloud = mesh.sample_points_uniformly(number_of_points=100000)

        # 定义点云文件的保存路径，以与STL文件相同的名字保存为点云文件（.ply）
        point_cloud_filename = os.path.splitext(filename)[0] + "_point_cloud.ply"
        point_cloud_save_path = os.path.join(point_cloud_save_directory, point_cloud_filename)

        # 保存点云文件
        o3d.io.write_point_cloud(point_cloud_save_path, point_cloud)

        # 打印保存信息
        print(f"点云已保存至：{point_cloud_save_path}")

        # 显示点云
        o3d.visualization.draw_geometries([point_cloud], window_name="点云显示 - 从STL生成")


