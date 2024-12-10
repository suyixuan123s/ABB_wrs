"""
Author: Yixuan Su
Date: 2024/11/19 21:32
File: object_pose_estimation.py
Description: 
"""

import os
import numpy as np
import open3d as o3d

# 定义文件路径
stl_directory = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Task4_ICP_GOFA5\meshes"
scene_point_cloud_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB_Point_Cloud\colored_point_cloud1118.ply"

# 加载场景点云
scene_point_cloud = o3d.io.read_point_cloud(scene_point_cloud_path)

# 设置最大配准距离（以确定匹配的准确度）
max_correspondence_distance = 0.02

# 遍历文件夹中的所有 STL 文件，逐个进行 ICP_Iterative_Closest_Point 配准
for stl_filename in os.listdir(stl_directory):
    if stl_filename.endswith(".STL"):
        # 加载 STL 文件并生成点云
        stl_path = os.path.join(stl_directory, stl_filename)
        mesh = o3d.io.read_triangle_mesh(stl_path)
        object_point_cloud = mesh.sample_points_uniformly(number_of_points=100000)

        # 使用 ICP_Iterative_Closest_Point 算法将物体点云与场景点云配准
        trans_init = np.eye(4)
        reg_p2p = o3d.pipelines.registration.registration_icp(
            object_point_cloud, scene_point_cloud, max_correspondence_distance, init=trans_init,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
        )

        transformation_matrix = reg_p2p.transformation
        print(f"物体 {stl_filename} 的位姿：")
        print(transformation_matrix)

        # 应用变换矩阵到物体点云
        object_transformed = object_point_cloud.transform(transformation_matrix)

        # 计算物体在场景点云中的对应点，并将它们移除
        distances = scene_point_cloud.compute_point_cloud_distance(object_transformed)
        distances = np.asarray(distances)

        # 设置阈值以确定哪些点属于匹配的物体
        threshold = 0.005  # 根据实验设置合理的阈值
        indices_to_remove = np.where(distances < threshold)[0]

        # 使用 select_by_index 来移除匹配点
        scene_point_cloud = scene_point_cloud.select_by_index(indices_to_remove, invert=True)

# 保存最终的场景点云
output_scene_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB_Point_Cloud\colored_point_cloud1118.ply"
o3d.io.write_point_cloud(output_scene_path, scene_point_cloud)

print(f"最终的场景点云已保存至：{output_scene_path}")
