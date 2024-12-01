"""
Author: Yixuan Su
Date: 2024/11/17 10:26
File: single_stl_to_point_cloud.py
"""

import os
import open3d as o3d

# 定义文件路径
stl_file_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Task4_ICP_GOFA5\meshes\rack_5ml_green.STL"
point_cloud_directory = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Task4_ICP_GOFA5\point_cloud"

# 读取STL文件
mesh = o3d.io.read_triangle_mesh(stl_file_path)
if mesh.is_empty():
    raise FileNotFoundError(f"无法加载STL文件：{stl_file_path}")

# 将三角网格转化为点云
point_cloud = mesh.sample_points_uniformly(number_of_points=100000)

# 获取原始 STL 文件的文件名并构造点云文件名
filename_no_ext = os.path.splitext(os.path.basename(stl_file_path))[0]  # 获取无扩展名的文件名
point_cloud_filename = f"{filename_no_ext}_point_cloud.ply"  # 添加 '_point_cloud.ply' 后缀
point_cloud_save_path = os.path.join(point_cloud_directory, point_cloud_filename)  # 组合成完整路径

# 创建保存点云的目录（如果不存在）
os.makedirs(point_cloud_directory, exist_ok=True)

# 保存点云文件
o3d.io.write_point_cloud(point_cloud_save_path, point_cloud)

# 显示点云
o3d.visualization.draw_geometries([point_cloud], window_name="点云显示 - 从STL生成")

print(f"点云已保存至：{point_cloud_save_path}")
