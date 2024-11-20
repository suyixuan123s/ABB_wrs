"""
Author: Yixuan Su
Date: 2024/11/19 21:40
File: single_object_pose_estimation.py
Description: 
"""
import os
import numpy as np
import open3d as o3d

# 定义文件路径
stl_file_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Task4_ICP_GOFA5\meshes\rack_10ml_green.STL"
scene_point_cloud_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB_Point_Cloud\colored_point_cloud1118.ply"

# 加载场景点云
scene_point_cloud = o3d.io.read_point_cloud(scene_point_cloud_path)

# 加载 STL 文件并生成点云
mesh = o3d.io.read_triangle_mesh(stl_file_path)
object_point_cloud = mesh.sample_points_uniformly(number_of_points=100000)

# 设置最大配准距离（以确定匹配的准确度）
max_correspondence_distance = 0.5

# 使用 ICP 算法将物体点云与场景点云配准
trans_init = np.eye(4)
reg_p2p = o3d.pipelines.registration.registration_icp(
    object_point_cloud, scene_point_cloud, max_correspondence_distance, init=trans_init,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
)

# 打印物体的最终变换矩阵
transformation_matrix = reg_p2p.transformation
print(f"物体 {os.path.basename(stl_file_path)} 的位姿：")
print(transformation_matrix)

# 将变换应用到物体点云（可选，用于可视化变换结果）
object_transformed = object_point_cloud.transform(transformation_matrix)

# 可视化原始场景点云与配准后的物体点云
scene_point_cloud.paint_uniform_color([0.8, 0.8, 0.8])  # 场景点云设置为灰色
object_transformed.paint_uniform_color([1, 0, 0])  # 物体点云设置为红色

o3d.visualization.draw_geometries([scene_point_cloud, object_transformed], window_name="物体配准结果")

# 保存配准后的物体点云（可选）
output_transformed_object_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB_Point_Cloud\rack_5ml_green_transformed.ply"
o3d.io.write_point_cloud(output_transformed_object_path, object_transformed)
print(f"配准后的物体点云已保存至：{output_transformed_object_path}")
