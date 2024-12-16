"""
Author: Yixuan Su
Date: 2024/12/15 18:21
File: read_triangle_mesh.py
Description: 
"""
import os
import open3d as o3d
import numpy as np


def normalize_point_cloud(point_cloud):
    # 获取点云坐标
    points = np.asarray(point_cloud.points)

    # 计算点云的边界框
    min_coords = np.min(points, axis=0)
    max_coords = np.max(points, axis=0)

    # 计算边界框的尺度（最大值和最小值之间的差距）
    scale = np.max(max_coords - min_coords)

    # 归一化：将点云缩放到 [0, 1] 范围
    normalized_points = (points - min_coords) / scale

    # 更新点云的坐标
    point_cloud.points = o3d.utility.Vector3dVector(normalized_points)

    return point_cloud


# 定义文件路径
stl_file_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task3_ICP_pre-processing\temp.stl"
point_cloud_directory = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task6_Point_cloud_processing\Dataset_Point_Cloud"

# 读取STL文件
mesh = o3d.io.read_triangle_mesh(stl_file_path)
if mesh.is_empty():
    raise FileNotFoundError(f"无法加载STL文件：{stl_file_path}")

# 获取三角网格的顶点
vertices = np.asarray(mesh.vertices)

# 获取原始 STL 文件的文件名并构造点云文件名
filename_no_ext = os.path.splitext(os.path.basename(stl_file_path))[0]  # 获取无扩展名的文件名
point_cloud_filename = f"{filename_no_ext}_point_cloud.ply"  # 添加 '_point_cloud.ply' 后缀
point_cloud_save_path = os.path.join(point_cloud_directory, point_cloud_filename)  # 组合成完整路径

# 创建保存点云的目录（如果不存在）
os.makedirs(point_cloud_directory, exist_ok=True)

# 将三角网格的顶点转换为Open3D点云
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(vertices)

# 保存点云文件
o3d.io.write_point_cloud(point_cloud_save_path, point_cloud)

# 显示点云
o3d.visualization.draw_geometries([point_cloud], window_name="点云显示 - 从STL生成")

print(f"点云已保存至：{point_cloud_save_path}")


stl_point_cloud_normalized = normalize_point_cloud(point_cloud)

# 可视化归一化后的 STL 点云和 Realsense 点云（查看大小是否一致）
o3d.visualization.draw_geometries([stl_point_cloud_normalized])

# 保存归一化后的 STL 点云
stl_output_file = os.path.join(point_cloud_directory, "normalized_stl_point_cloud.ply")
if o3d.io.write_point_cloud(stl_output_file, stl_point_cloud_normalized):
    print(f"STL 点云成功保存到: {stl_output_file}")
else:
    print(f"保存 STL 点云失败: {stl_output_file}")