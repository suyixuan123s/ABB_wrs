"""
Author: Yixuan Su
Date: 2024/11/19 10:23
File: ICP_initial_guess.py
"""

import open3d as o3d
import numpy as np

# 加载点云数据
# source = o3d.io.read_point_cloud("source_point_cloud.ply")  # 读取源点云
# target = o3d.io.read_point_cloud("target_point_cloud.ply")  # 读取目标点云


source = o3d.io.read_point_cloud(
    r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task6_Point_cloud_processing\Dataset_Point_Cloud\normalized_realsense_point_cloud.ply")  # 读取源点云
target = o3d.io.read_point_cloud(
    r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task8_STL_To_Point_Cloud\Datasets\stl_to_point_cloud_voxelization_normalized.ply")  # 读取目标点云

# source = o3d.io.read_point_cloud(r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\ICP_Iterative_Closest_Point\colored_point_cloud120302.ply")  # 读取源点云
# target = o3d.io.read_point_cloud(r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\ICP_Iterative_Closest_Point\converted_stl_point_cloud.ply")  # 读取目标点云

# 设置不同颜色以区分点云
source.paint_uniform_color([1, 0, 0])  # 红色
target.paint_uniform_color([0, 1, 0])  # 绿色

# 可视化原始点云（可选）
print("显示源点云...")
o3d.visualization.draw_geometries([source], window_name="source Registration", width=1024, height=768)

# 可视化原始点云（可选）
print("显示目标点云...")
o3d.visualization.draw_geometries([target], window_name="target Registration", width=1024, height=768)

# 设定一个初始粗对齐变换矩阵（接近于我们之前施加的旋转和平移）
# initial_guess = np.array([
#     [0.51058156, 0.00491067, 0.8598153, 0.30657232],
#     [-0.0921574, 0.99453585, 0.04904545, -0.30394721],
#     [-0.8548763, -0.10428005, 0.50824422, 1.47714186],
#     [0, 0, 0, 1]
# ])

# initial_guess = np.array([[-0.69143706, -0.71064714,  0.12998249, 1.00587988],
#                           [-0.64461358, 0.52565226, -0.55512434,  0.91515372],
#                           [0.32617193, -0.46762202, -0.82154825, 0.44813821],
#                           [0, 0, 0, 1]
#                           ])

#
# initial_guess = np.array([[0.82649164, -0.06160907, -0.55956759, 1.24135564],
#                           [0.56273542, 0.06304022, 0.82422981, 2.50385857],
#                           [-0.01550476, -0.99610756, 0.08677182, 0.82665619],
#                           [0, 0, 0, 1]])


initial_guess = np.array([[-0.82647682, -0.06199939, 0.55954637, 0.78003854],
                          [-0.56274889, 0.06308269, -0.82421736, 1.22544392],
                          [0.01580328, -0.99608065, -0.08702647, 0.77418869],
                          [0, 0, 0, 1]])
# # 初始粗对齐（可以提供一个初步的旋转和平移）
# # 这里假设初始对齐为单位矩阵（即没有旋转和平移）
# trans_init = np.eye(4)  # 初始变换矩阵（单位矩阵）

# 使用 ICP_Iterative_Closest_Point 算法进行点云配准
# pipelines.registration 模块在较新版本的 Open3D 中适用

max_correspondence_distance = 0.5  # 最大匹配距离

reg_icp = o3d.pipelines.registration.registration_icp(
    source, target, max_correspondence_distance, init=initial_guess,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    # 使用点到点 ICP_Iterative_Closest_Point
    criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
        max_iteration=3000,  # 最大迭代次数
        relative_fitness=1e-6,  # 相对收敛条件
        relative_rmse=1e-6  # 相对均方根误差收敛条件
    )
)

# 打印配准结果
# print("ICP_Iterative_Closest_Point converged:", reg_icp.converged)
print("Fitness:", reg_icp.fitness)  # 配准度（匹配的点对数量比例，越接近 1 越好）
print("RMSE:", reg_icp.inlier_rmse)  # 配准误差（越小越好）

# 打印最终变换矩阵
print(f"Transformation Matrix:")
print(reg_icp.transformation)  # 最终变换矩阵

# 应用变换到源点云，得到对齐后的点云
source_pcd = source.transform(reg_icp.transformation)

# 可视化对齐后的点云
o3d.visualization.draw_geometries([source, target], window_name="After ICP_Iterative_Closest_Point Registration",
                                  width=1024, height=768)
