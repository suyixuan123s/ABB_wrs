"""
Author: Yixuan Su
Date: 2024/11/25 15:02
File: point_cloud_analyze.py
Description: 
"""
import open3d as o3d


# 加载点云文件
def load_point_cloud(file_path):
    point_cloud = o3d.io.read_point_cloud(file_path)
    return point_cloud


# 分析点云的点数
def analyze_point_cloud(point_cloud, description="Point Cloud"):
    num_points = len(point_cloud.points)
    print(f"{description}:")
    print(f"Number of points: {num_points}")
    print(f"Bounds (min, max): {point_cloud.get_min_bound()}, {point_cloud.get_max_bound()}")
    return num_points


# 示例使用
if __name__ == "__main__":
    # 替换为你的点云文件路径
    scene_file = "../ABB/Task3_ICP_learning/point_cloud.ply"
    # object_file = "object.ply"

    # 加载点云
    scene_pcd = load_point_cloud(scene_file)
    # object_pcd = load_point_cloud(object_file)

    # 分析点云
    print("\nAnalyzing Scene Point Cloud:")
    analyze_point_cloud(scene_pcd, "Scene Point Cloud")

    # print("\nAnalyzing Object Point Cloud:")
    # analyze_point_cloud(object_pcd, "Object Point Cloud")

    # 可视化点云
    print("\nVisualizing the point clouds...")
    o3d.visualization.draw_geometries([scene_pcd])
