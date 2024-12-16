"""
Author: Yixuan Su
Date: 2024/11/25 15:22
File: ICP_eye4_voxel_down_sample_estimate_normals_ransac.py
Description: 
"""
import open3d as o3d
import numpy as np

# 加载点云
scene_pcd = o3d.io.read_point_cloud("scene.ply")  # 场景点云
object_pcd = o3d.io.read_point_cloud("object.ply")  # 目标物体点云

# 点云预处理下采样
scene_pcd = scene_pcd.voxel_down_sample(voxel_size=0.02)
object_pcd = object_pcd.voxel_down_sample(voxel_size=0.02)

# 估计法线
object_pcd.estimate_normals()
scene_pcd.estimate_normals()

# 计算FPFH特征
object_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
    object_pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))
scene_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
    scene_pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))

# 粗配准
result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
    object_pcd, scene_pcd, object_fpfh, scene_fpfh,
    max_correspondence_distance=0.1,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
    ransac_n=4,
    checkers=[o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
              o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(0.1)],
    criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))

# 精配准
result_icp = o3d.pipelines.registration.registration_icp(
    object_pcd, scene_pcd, max_correspondence_distance=0.02,
    init=result_ransac.transformation,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint())

# 变换矩阵（目标物体相对于相机）
transformation = result_icp.transformation
print("Transformation Matrix:\n", transformation)

# 可视化
object_pcd.transform(transformation)
o3d.visualization.draw_geometries([scene_pcd, object_pcd])
