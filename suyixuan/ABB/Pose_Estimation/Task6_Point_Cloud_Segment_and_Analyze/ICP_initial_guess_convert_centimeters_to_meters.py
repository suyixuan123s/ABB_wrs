"""
Author: Yixuan Su
Date: 2024/11/19 10:23
File: ICP_initial_guess_convert_centimeters_to_meters.py
"""

import open3d as o3d
import numpy as np

# 加载点云数据
source = o3d.io.read_point_cloud(r"/suyixuan/ABB/ICP_Iterative_Closest_Point\colored_point_cloud120302.ply")  # 读取源点云
target = o3d.io.read_point_cloud(r"/suyixuan/ABB/ICP_Iterative_Closest_Point\converted_stl_point_cloud.ply")  # 读取目标点云

# 确保两者单位一致
# 将目标点云（STL点云）从厘米转换为米
def convert_centimeters_to_meters(point_cloud):
    points = np.asarray(point_cloud.points)
    points = points / 100.0  # 厘米转米
    point_cloud.points = o3d.utility.Vector3dVector(points)
    return point_cloud

# 将目标点云单位转换
target = convert_centimeters_to_meters(target)

# 设置不同颜色以区分点云
source.paint_uniform_color([1, 0, 0])  # 红色
target.paint_uniform_color([0, 1, 0])  # 绿色

# 可视化原始点云（可选）
print("显示源点云...")
o3d.visualization.draw_geometries([source], window_name="source Registration", width=1024, height=768)

print("显示目标点云...")
o3d.visualization.draw_geometries([target], window_name="target Registration", width=1024, height=768)

# 设定一个初始粗对齐变换矩阵
initial_guess = np.array([
    [0.51058156, 0.00491067, 0.8598153, 0.30657232],
    [-0.0921574, 0.99453585, 0.04904545, -0.30394721],
    [-0.8548763, -0.10428005, 0.50824422, 1.47714186],
    [0, 0, 0, 1]
])

# 使用 ICP_Iterative_Closest_Point 算法进行点云配准
max_correspondence_distance = 0.5  # 最大匹配距离

reg_icp = o3d.pipelines.registration.registration_icp(
    source, target, max_correspondence_distance, init=initial_guess,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
        max_iteration=3000,
        relative_fitness=1e-6,
        relative_rmse=1e-6
    )
)

# 打印配准结果
print("Fitness:", reg_icp.fitness)
print("RMSE:", reg_icp.inlier_rmse)
print(f"Transformation Matrix:\n{reg_icp.transformation}")

# 应用变换到源点云，得到对齐后的点云
source.transform(reg_icp.transformation)

# 可视化对齐后的点云
o3d.visualization.draw_geometries([source, target], window_name="After ICP_Iterative_Closest_Point Registration", width=1024, height=768)
