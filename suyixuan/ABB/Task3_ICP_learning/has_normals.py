"""
Author: Yixuan Su
Date: 2024/11/25 15:23
File: has_normals.py
Description: 
"""
import open3d as o3d

# 加载点云
point_cloud = o3d.io.read_point_cloud("../Task4_ICP_GOFA5/stl_to_point_cloud_dataset/rack_5ml_green_point_cloud.ply")  # 替换为你的点云文件路径

# 检查点云是否有法线
if point_cloud.has_normals():
    print("The point cloud contains normal information.")
else:
    print("The point cloud does NOT contain normal information.")




