"""
Author: Yixuan Su
Date: 2024/11/25 13:49
File: ICP_eye4_voxel_down_sample_estimate_normals.py
Description: 
"""

import os
import open3d as o3d
import numpy as np

# 加载点云文件
source_ply_path = r'/suyixuan/ABB/ICP_Iterative_Closest_Point\colored_point_cloud120302.ply'  # 分割生成的点云
target_ply_path = r'/suyixuan/ABB/ICP_Iterative_Closest_Point\box_point_cloud.ply'  # 目标点云（STL转换）

# 加载点云
source = o3d.io.read_point_cloud(source_ply_path)
target = o3d.io.read_point_cloud(target_ply_path)

# 设置点云颜色
source.paint_uniform_color([1, 0, 0])  # 红色
target.paint_uniform_color([0, 1, 0])  # 绿色

# 可视化初始状态
print("显示初始点云...")
o3d.visualization.draw_geometries([source], window_name="source Alignment")

# 可视化初始状态
print("显示初始点云...")
o3d.visualization.draw_geometries([target], window_name="target Alignment")

# 对点云进行下采样以提高匹配效率
source = source.voxel_down_sample(voxel_size=0.005)
target = target.voxel_down_sample(voxel_size=0.005)

# 计算法线
source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))

# 执行点云配准（ICP_Iterative_Closest_Point）
print("开始点云对齐...")
threshold = 0.02  # 设置匹配阈值（单位：米）
initial_transform = np.eye(4)  # 初始变换矩阵（可设置为先验估计）


# 使用 ICP_Iterative_Closest_Point 进行点云匹配
reg_p2p = o3d.pipelines.registration.registration_icp(
    source, target, threshold, initial_transform,
    o3d.pipelines.registration.TransformationEstimationPointToPoint()
)

# 获取匹配后的变换矩阵
transformation_matrix = reg_p2p.transformation
print("ICP_Iterative_Closest_Point 配准完成。")
print("变换矩阵：\n", transformation_matrix)


# 将源点云应用变换矩阵后显示对齐结果
source.transform(transformation_matrix)
print("显示对齐后的点云...")
o3d.visualization.draw_geometries([source, target], window_name="Aligned Point Clouds")

# 保存对齐结果（可选）
aligned_ply_path = "aligned_source.ply"
# os.makedirs(os.path.dirname(aligned_ply_path), exist_ok=True)
o3d.io.write_point_cloud(aligned_ply_path, source)
print(f"对齐后的源点云已保存到: {aligned_ply_path}")
