"""
Author: Yixuan Su
Date: 2024/11/19 10:23
File: ICP_registration2.py
"""

import open3d as o3d
import numpy as np

# 加载点云数据
# source = o3d.io.read_point_cloud("source_point_cloud.ply")  # 读取源点云
# target = o3d.io.read_point_cloud("target_point_cloud.ply")  # 读取目标点云


source = o3d.io.read_point_cloud(
    r"/suyixuan/ABB/ICP_Iterative_Closest_Point\normalized_realsense_point_cloud.ply")  # 读取源点云
target = o3d.io.read_point_cloud(
    r"/suyixuan/ABB/ICP_Iterative_Closest_Point\normalized_stl_point_cloud.ply")  # 读取目标点云

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

# initial_guess = np.array([
#     [0.99962989, -0.00844353, 0.0258611, 0.00203317],
#     [0.01180651, 0.9910745, -0.13278532, -0.09554107],
#     [-0.0245091, 0.1330415, 0.99080738, 0.62364565],
#     [0, 0, 0, 1]
# ])

initial_guess = np.array(

    [[9.98695073e-01, -4.29472256e-03, -5.08892267e-02, -1.89501654e-02],
     [2.83316108e-04, 9.96908356e-01, -7.85725674e-02, -2.95379411e-02],
     [5.10693429e-02, 7.84556141e-02, 9.95608678e-01, -1.87374895e-02],
     [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
     ])

# # 初始粗对齐（可以提供一个初步的旋转和平移）
# # 这里假设初始对齐为单位矩阵（即没有旋转和平移）
# trans_init = np.eye(4)  # 初始变换矩阵（单位矩阵）

# 使用 ICP_Iterative_Closest_Point 算法进行点云配准
# pipelines.registration 模块在较新版本的 Open3D 中适用
max_correspondence_distance = 0.5  # 最大匹配距离

reg_icp = o3d.pipelines.registration.registration_icp(
    source, target, max_correspondence_distance, init=initial_guess,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),  # 使用点到点 ICP_Iterative_Closest_Point
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
source.transform(reg_icp.transformation)

# 可视化对齐后的点云
o3d.visualization.draw_geometries([source, target], window_name="After ICP_Iterative_Closest_Point Registration", width=1024, height=768)

'''
Fitness: 1.0
RMSE: 0.00029624882518490526
Transformation Matrix:

[[ 5.82907243e-01 -7.07109247e-01 -4.00269483e-01  1.00016291e+00]
 [ 5.82925787e-01  7.07104315e-01 -4.00251189e-01 -1.64462761e-04]
 [ 5.66053595e-01 -1.80858771e-05  8.24368441e-01  2.23939945e-06]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]


'''
