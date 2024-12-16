"""
Author: Yixuan Su
Date: 2024/11/18 13:56
File: segment_object_point_cloud.py

"""

import os
import open3d as o3d
import numpy as np

def segment_objects_DBSCAN(point_cloud, eps=0.02, min_points=50):
    """
    使用DBSCAN对点云进行聚类分割。
    """
    labels = np.array(point_cloud.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))
    max_label = labels.max()

    object_clouds = []
    for i in range(max_label + 1):
        indices = np.where(labels == i)[0]
        object_cloud = point_cloud.select_by_index(indices)
        object_clouds.append(object_cloud)

    print(f"DBSCAN检测到 {max_label + 1} 个物体")
    return object_clouds

def segment_with_region_growing(point_cloud, nb_neighbors=30, smoothness_threshold=0.3):
    """
    使用区域增长方法分割点云

    参数：
        point_cloud_dataset (open3d.geometry.PointCloud): 输入的点云对象。
        nb_neighbors (int): 邻域的数量，用于法向量估计。
        smoothness_threshold (float): 控制区域的平滑性。角度越小，分割的区域越严格。

    返回：
        object_clouds (list): 各个分割区域的点云列表。
    """
    # Step 1: 估计法向量
    point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=nb_neighbors))

    # Step 2: 初始化KDTree用于邻域搜索
    kdtree = o3d.geometry.KDTreeFlann(point_cloud)

    # Step 3: 初始化区域生长参数
    labels = np.full((len(point_cloud.points),), -1, dtype=int)
    current_label = 0

    for idx in range(len(point_cloud.points)):
        # 跳过已标记点
        if labels[idx] != -1:
            continue

        # 新建一个簇
        queue = [idx]
        labels[idx] = current_label

        # 使用广度优先搜索进行区域生长
        while queue:
            point_idx = queue.pop(0)
            # 在KDTree中搜索邻居
            [_, neighbors, _] = kdtree.search_radius_vector_3d(point_cloud.points[point_idx], nb_neighbors)

            for neighbor_idx in neighbors:
                if labels[neighbor_idx] == -1:  # 如果邻居未被标记
                    # 计算法向量之间的角度
                    angle = np.dot(point_cloud.normals[point_idx], point_cloud.normals[neighbor_idx])
                    if angle > smoothness_threshold:  # 判断法向量是否相似
                        labels[neighbor_idx] = current_label
                        queue.append(neighbor_idx)

        current_label += 1

    # Step 4: 分割为点云区域
    object_clouds = []
    for label in range(current_label):
        indices = np.where(labels == label)[0]
        object_cloud = point_cloud.select_by_index(indices)
        object_clouds.append(object_cloud)

    print(f"区域增长检测到 {current_label} 个区域")
    return object_clouds


def segment_with_ransac(point_cloud, distance_threshold=0.01, ransac_n=3, num_iterations=1000):
    """
    使用 RANSAC 进行平面分割

    参数：
        point_cloud_dataset (open3d.geometry.PointCloud): 输入的点云。
        distance_threshold (float): 一个点与平面之间的最大距离阈值。若点与平面的距离小于该值，则视为属于该平面。
        ransac_n (int): 每次随机采样的点数。对于平面检测通常设置为3个点。
        num_iterations (int): RANSAC的迭代次数，决定了找到最佳平面的概率。

    返回：
        plane_cloud (open3d.geometry.PointCloud): 从点云中提取的平面。
        remaining_cloud (open3d.geometry.PointCloud): 除去平面后剩余的点云。
    """


    plane_model, inliers = point_cloud.segment_plane(distance_threshold=distance_threshold,
                                                     ransac_n=ransac_n,
                                                     num_iterations=num_iterations)
    # 获取平面上的点集
    plane_cloud = point_cloud.select_by_index(inliers)

    # 获取剩余的非平面点集
    remaining_cloud = point_cloud.select_by_index(inliers, invert=True)

    print("RANSAC检测到一个平面")
    return plane_cloud, remaining_cloud

def segment_with_voxel(point_cloud, voxel_size=0.05):
    """
    使用体素分割方法聚类点云
    """
    voxel_down_sampled = point_cloud.voxel_down_sample(voxel_size)
    print("体素下采样完成")
    return voxel_down_sampled

def save_segmented_objects(object_clouds, method_name="method"):
    """
    保存分割的物体点云，每种分割方法的结果保存在单独的文件夹中。
    """
    # 创建保存分割结果的目录
    base_dir = f'/suyixuan/ABB/ABB-12-03/Dataset/segmentation_results/{method_name}'
    os.makedirs(base_dir, exist_ok=True)

    # 保存每个物体点云到该方法的文件夹中
    for i, obj_cloud in enumerate(object_clouds):
        save_path = os.path.join(base_dir, f'{method_name}_object_{i}_segmented.ply')
        o3d.io.write_point_cloud(save_path, obj_cloud)
        print(f"物体 {i} 的分割点云已保存为 {save_path}")

if __name__ == "__main__":
    # 加载裁剪后的点云文件路径
  #  cropped_point_cloud_path = r'E:\ABB-Project\ABB_wrs\ABBB\ABB-11-6\Dataset\colored_point_cloud1106.ply'
    cropped_point_cloud_path = r'E:\ABB-Project\ABB_wrs\suyixuan\ABB\ICP\cropped_point_cloud1203.ply'

    # 读取裁剪后的点云
    point_cloud = o3d.io.read_point_cloud(cropped_point_cloud_path)
    if point_cloud.is_empty():
        print("加载点云失败，请检查文件路径。")
        exit(1)

    # 手动选择分割方法
    print("请选择分割方法：")
    print("1. DBSCAN 分割")
    print("2. 区域增长分割")
    print("3. RANSAC 平面分割")
    print("4. 体素下采样分割")
    choice = input("输入方法编号（1-4）:")

    if choice == "1":
        # DBSCAN 分割
        object_clouds_dbscan = segment_objects_DBSCAN(point_cloud, eps=0.02, min_points=50)
        save_segmented_objects(object_clouds_dbscan, method_name="DBSCAN")

    elif choice == "2":
        # 区域增长分割
        object_clouds_region = segment_with_region_growing(point_cloud)
        save_segmented_objects(object_clouds_region, method_name="RegionGrowing")

    elif choice == "3":
        # RANSAC 平面分割
        plane_cloud, remaining_cloud = segment_with_ransac(point_cloud)
        ransac_dir = '/suyixuan/ABB/ABB-12-03/Dataset/segmentation_results/RANSAC'
        os.makedirs(ransac_dir, exist_ok=True)
        o3d.io.write_point_cloud(os.path.join(ransac_dir, "plane_segmented.ply"), plane_cloud)
        print("平面分割结果已保存")
        # 保存平面外的点云部分
        save_segmented_objects([remaining_cloud], method_name="RANSAC_remaining")

    elif choice == "4":
        # 体素分割
        voxel_cloud = segment_with_voxel(point_cloud)
        voxel_dir = '/suyixuan/ABB/ABB-12-03/Dataset/segmentation_results/Voxel'
        os.makedirs(voxel_dir, exist_ok=True)
        o3d.io.write_point_cloud(os.path.join(voxel_dir, "voxel_segmented.ply"), voxel_cloud)
        print("体素分割结果已保存")

    else:
        print("无效输入，请选择 1-4 的数字。")
