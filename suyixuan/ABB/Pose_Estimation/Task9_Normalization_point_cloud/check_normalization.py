"""
Author: Yixuan Su
Date: 2024/12/16 18:46
File: check_normalization.py
Description: 
"""


import open3d as o3d
import numpy as np


def check_normalization(point_cloud, name="Point Cloud"):
    """
    检查点云的归一化效果，打印点云的边界框尺寸
    """
    # 获取点云的坐标
    points = np.asarray(point_cloud.points)

    # 计算边界框的最小值和最大值
    bbox_min = np.min(points, axis=0)
    bbox_max = np.max(points, axis=0)

    # 输出边界框信息
    print(f"--- {name} ---")
    print(f"Bounding Box Min: {bbox_min}")
    print(f"Bounding Box Max: {bbox_max}")
    print(f"Range in X: {bbox_max[0] - bbox_min[0]:.6f}")
    print(f"Range in Y: {bbox_max[1] - bbox_min[1]:.6f}")
    print(f"Range in Z: {bbox_max[2] - bbox_min[2]:.6f}\n")


# 加载两个点云文件
source_ply_path = r"/suyixuan/ABB/Pose_Estimation/Task6_Point_Cloud_Segment_and_Analyze/Dataset_Point_Cloud/normalized_realsense_point_cloud.ply"  # 修改为实际路径
target_ply_path = r"/suyixuan/ABB/Pose_Estimation/Task8_STL_To_Point_Cloud/Datasets/stl_to_point_cloud_voxelization_normalized.ply"  # 修改为实际路径

# 加载点云
source = o3d.io.read_point_cloud(source_ply_path)
target = o3d.io.read_point_cloud(target_ply_path)

# 检查归一化效果
check_normalization(source, name="Source Point Cloud")
check_normalization(target, name="Target Point Cloud")




# 可视化两个点云
print("可视化两个点云...")
o3d.visualization.draw_geometries([source], window_name="Source Point Cloud")
o3d.visualization.draw_geometries([target], window_name="Target Point Cloud")





"""
E:\ABB-Project\ABB_wrs\venv\Scripts\python.exe E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task8_STL_To_Point_Cloud\check_normalization.py 
--- Source Point Cloud ---
Bounding Box Min: [0. 0. 0.]
Bounding Box Max: [1.         0.79720255 0.81315094]
Range in X: 1.000000
Range in Y: 0.797203
Range in Z: 0.813151

--- Target Point Cloud ---
Bounding Box Min: [0. 0. 0.]
Bounding Box Max: [1.         0.73643411 0.69767442]
Range in X: 1.000000
Range in Y: 0.736434
Range in Z: 0.697674

可视化两个点云...
Process finished with exit code 0



"""