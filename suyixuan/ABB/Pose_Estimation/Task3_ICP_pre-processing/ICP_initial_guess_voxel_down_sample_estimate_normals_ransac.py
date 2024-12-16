"""
Author: Yixuan Su
Date: 2024/11/25 15:22
File: ICP_eye4_voxel_down_sample_and_estimate_normals_and_fpfh_Ransac1.py
Description: 
"""

import open3d as o3d
import numpy as np

# 加载点云文件
source_ply_path = r'E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task6_Point_cloud_processing\Dataset_Point_Cloud\normalized_realsense_point_cloud.ply'  # 分割生成的点云
target_ply_path = r'E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task8_STL_To_Point_Cloud\Datasets\stl_to_point_cloud_voxelization_normalized.ply'  # 目标点云（STL转换）

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
# 计算FPFH特征
object_fpfh = o3d.pipelines.registration.compute_fpfh_feature(target, o3d.geometry.KDTreeSearchParamHybrid(radius=0.05,
                                                                                                           max_nn=30))
scene_fpfh = o3d.pipelines.registration.compute_fpfh_feature(source, o3d.geometry.KDTreeSearchParamHybrid(radius=0.05,
                                                                                                          max_nn=30))

# 打印特征维度
print("Object FPFH feature dimensions:", object_fpfh.data.shape)
print("Scene FPFH feature dimensions:", scene_fpfh.data.shape)

# 粗配准
result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
    target, source, object_fpfh, scene_fpfh,
    max_correspondence_distance=0.1,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
    ransac_n=3, checkers=[o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                          o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(0.1)],
    criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))

# 精配准
result_icp = o3d.pipelines.registration.registration_icp(
    target, source, max_correspondence_distance=0.02,
    init=result_ransac.transformation,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
        max_iteration=3000,  # 最大迭代次数
        relative_fitness=1e-6,  # 相对收敛条件
        relative_rmse=1e-6  # 相对均方根误差收敛条件
    )
)

# 打印配准结果
# print("ICP_Iterative_Closest_Point converged:", reg_icp.converged)
print("Fitness:", result_icp.fitness)  # 配准度（匹配的点对数量比例，越接近 1 越好）
print("RMSE:", result_icp.inlier_rmse)  # 配准误差（越小越好）

# 打印最终变换矩阵
print(f"Transformation Matrix:")
print(result_icp.transformation)  # 最终变换矩阵

# 应用变换到源点云，得到对齐后的点云
source.transform(result_icp.transformation)

# 可视化对齐后的点云
o3d.visualization.draw_geometries([source, target], window_name="After ICP_Iterative_Closest_Point Registration",
                                  width=1024, height=768)
