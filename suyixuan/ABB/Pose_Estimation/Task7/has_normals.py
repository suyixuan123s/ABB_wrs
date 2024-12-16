"""
Author: Yixuan Su
Date: 2024/12/16 16:17
File: has_normals.py
Description: 
"""
import open3d as o3d

# 加载点云文件
point_cloud = o3d.io.read_point_cloud(r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task6_Point_cloud_processing\Dataset_Point_Cloud\normalized_realsense_point_cloud.ply")

# 检查点云是否包含法线
if point_cloud.has_normals():
    print("点云包含法线")
else:
    print("点云不包含法线")
