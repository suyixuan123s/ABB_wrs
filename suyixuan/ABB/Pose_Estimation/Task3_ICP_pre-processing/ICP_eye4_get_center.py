"""
Author: Yixuan Su
Date: 2024/11/26 20:52
File: ICP_eye4_get_center.py
Description: 
"""
import open3d as o3d
import numpy as np


# 加载相机拍摄的点云
def load_camera_point_cloud(file_path):
    # 假设你已经有包含颜色信息的点云文件
    camera_pcd = o3d.io.read_point_cloud(file_path)
    return camera_pcd

# 主程序
if __name__ == "__main__":
    # STL文件路径
    stl_file = r"/suyixuan/ABB/Pose_Estimation/Task5_ICP_GOFA5/meshes/rack_5ml_green.STL"
    camera_pcd_file = r"point_cloud_dataset.ply"  # 替换为相机点云文件路径

    # 加载STL文件并生成点云
    stl_points = o3d.io.read_triangle_mesh(stl_file)
    stl_pcd = stl_points.sample_points_uniformly(number_of_points=5000)

    # 加载相机拍摄的点云
    camera_pcd = load_camera_point_cloud(camera_pcd_file)

    # 初步对齐：将两个点云的中心对齐
    stl_center = stl_pcd.get_center()
    camera_center = camera_pcd.get_center()

    translation_vector = stl_center - camera_center
    camera_pcd.translate(translation_vector)

    # ICP_Iterative_Closest_Point 配准
    threshold = 0.05  # 配准的距离阈值，可以根据点云稠密度调整
    transformation_init = np.eye(4)  # 初始变换矩阵

    reg_p2p = o3d.pipelines.registration.registration_icp(
        camera_pcd, stl_pcd, threshold, transformation_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())

    # fitness：表示配准的拟合度，衡量源点云与目标点云之间匹配的质量。fitness > 0.5 被视为配准收敛。
    # inlier_rmse：表示内点的均方根误差，即配准过程中源点云与目标点云对应点之间的平均距离。
    # transformation：输出最终的变换矩阵（4 x 4），表示如何将源点云对齐到目标点云。

    # 打印配准结果
    print("ICP_Iterative_Closest_Point converged:", reg_p2p.fitness > 0.5)
    print("Fitness:", reg_p2p.fitness)
    print("Inlier RMSE:", reg_p2p.inlier_rmse)
    print("Transformation matrix:\n", reg_p2p.transformation)

    # 应用变换并可视化配准结果
    camera_pcd.transform(reg_p2p.transformation)
    o3d.visualization.draw_geometries([stl_pcd, camera_pcd])
