"""
Author: Yixuan Su
Date: 2024/12/15 17:28
File: rearrange_points_by_distance.py
Description: 
"""
import numpy as np
import open3d as o3d


def load_stl_and_generate_point_cloud(stl_file_path):
    # 载入STL文件
    mesh = o3d.io.read_triangle_mesh(stl_file_path)
    # 提取顶点作为点云
    point_cloud = mesh.sample_points_uniformly(number_of_points=50000)
    return point_cloud


def calculate_point_distances(points):
    """
    计算点云中每两个点之间的距离，返回所有相邻点对之间的距离
    """
    distances = np.linalg.norm(points[:-1] - points[1:], axis=1)
    return distances


def rearrange_points_by_distance(point_cloud, target_distance):
    points = np.asarray(point_cloud.points)

    # 计算点云的边界框
    min_coords = np.min(points, axis=0)
    max_coords = np.max(points, axis=0)

    # 根据目标的平均距离，计算每行每列应有多少个点
    rows = int((max_coords[0] - min_coords[0]) / target_distance)
    cols = int((max_coords[1] - min_coords[1]) / target_distance)

    # 确保点云的行列数能尽量匹配点云的总数量
    total_points = rows * cols
    if len(points) < total_points:
        points = np.pad(points, ((0, total_points - len(points)), (0, 0)), mode='constant', constant_values=0)

    grid_points = []
    for row in range(rows):
        for col in range(cols):
            x = min_coords[0] + row * target_distance
            y = min_coords[1] + col * target_distance
            z = points[row * cols + col, 2]  # 使用原始点云中的z坐标

            grid_points.append([x, y, z])

    # 创建新的点云
    grid_points = np.array(grid_points)
    point_cloud.points = o3d.utility.Vector3dVector(grid_points)

    return point_cloud


def main():
    stl_file_path = r'E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task3_ICP_pre-processing\temp.stl'  # 替换为STL文件路径

    # 加载STL并生成点云
    point_cloud = load_stl_and_generate_point_cloud(stl_file_path)

    # 计算点云的相邻点距离（用于判断排列的距离）
    points = np.asarray(point_cloud.points)
    distances = calculate_point_distances(points)

    # 输出点云的一些统计信息
    print(f"平均距离: {np.mean(distances)}")
    print(f"最小距离: {np.min(distances)}")
    print(f"最大距离: {np.max(distances)}")

    # 按目标距离重新排列点云
    target_distance = 1.76  # 目标距离，根据你的需求进行调整
    rearranged_point_cloud = rearrange_points_by_distance(point_cloud, target_distance)

    # 可视化重新排列后的点云
    o3d.visualization.draw_geometries([rearranged_point_cloud])


if __name__ == "__main__":
    main()
