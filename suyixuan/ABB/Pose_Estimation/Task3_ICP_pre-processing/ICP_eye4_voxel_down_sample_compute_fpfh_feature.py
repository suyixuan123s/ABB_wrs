"""
Author: Yixuan Su
Date: 2024/11/25 15:22
File: ICP_eye4_voxel_down_sample_estimate_normals_ransac_orient_normals_towards_camera_location.py
Description:
"""

import open3d as o3d
import numpy as np

# 加载点云
source = o3d.io.read_point_cloud(r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task6_Point_cloud_processing\Dataset_Point_Cloud\normalized_realsense_point_cloud.ply")  # 读取源点云
target = o3d.io.read_point_cloud(r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task8_STL_To_Point_Cloud\Datasets\stl_to_point_cloud_voxelization_normalized.ply")  # 读取目标点云

# 设置点云颜色
source.paint_uniform_color([1, 0, 0])  # 红色
target.paint_uniform_color([0, 1, 0])  # 绿色

# 可视化原始点云（可选）
print("显示源点云...")
o3d.visualization.draw_geometries([source], window_name="source Registration", width=1024, height=768)

# 可视化原始点云（可选）
print("显示源点云...")
o3d.visualization.draw_geometries([target], window_name="target Registration", width=1024, height=768)

# 点云预处理下采样
voxel_size = 0.01
source = source.voxel_down_sample(voxel_size)
target = target.voxel_down_sample(voxel_size)

# 可视化下采样后的点云
print("显示源点云...")
o3d.visualization.draw_geometries([source], window_name="source voxel_down_sample Registration", width=1024, height=768)

# 可视化下采样后的点云
print("显示源点云...")
o3d.visualization.draw_geometries([target], window_name="target voxel_down_sample Registration", width=1024, height=768)

# 计算FPFH特征
source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
    source, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))
target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
    target, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))


# RANSAC粗配准
distance_threshold = voxel_size * 1.5
result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
    source, target, source_fpfh, target_fpfh, True,
    distance_threshold,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 4,
    [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
     o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
    o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))

# 应用RANSAC变换
source.transform(result_ransac.transformation)

# ICP精细配准
threshold = 0.02  # 设置匹配阈值（单位：米）
reg_p2p = o3d.pipelines.registration.registration_icp(
    source, target, threshold, result_ransac.transformation,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
        max_iteration=3000,  # 最大迭代次数
        relative_fitness=1e-6,  # 相对收敛条件
        relative_rmse=1e-6  # 相对均方根误差收敛条件
    )
)

# 获取最终变换矩阵
transformation_matrix = reg_p2p.transformation
print("最终变换矩阵：\n", transformation_matrix)

# 将源点云应用最终变换矩阵后显示对齐结果
source.transform(transformation_matrix)
o3d.visualization.draw_geometries([source, target], window_name="对齐后的点云")
