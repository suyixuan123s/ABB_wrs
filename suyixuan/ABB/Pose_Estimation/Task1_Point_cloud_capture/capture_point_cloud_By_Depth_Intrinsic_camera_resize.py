"""
Author: Yixuan Su
Date: 2024/11/18 13:56
File: capture_point_cloud_By_Depth_Intrinsic_transformation_simulation_resize.py
"""

import os
import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import cv2


def compute_transformation(alpha_deg, beta_deg, gamma_deg, tx, ty, tz):
    # 将角度转换为弧度
    alpha = np.radians(alpha_deg)
    beta = np.radians(beta_deg)
    gamma = np.radians(gamma_deg)

    # 构造旋转矩阵 (X, Y, Z 轴)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(alpha), -np.sin(alpha)],
        [0, np.sin(alpha), np.cos(alpha)]
    ])

    Ry = np.array([
        [np.cos(beta), 0, np.sin(beta)],
        [0, 1, 0],
        [-np.sin(beta), 0, np.cos(beta)]
    ])

    Rz = np.array([
        [np.cos(gamma), -np.sin(gamma), 0],
        [np.sin(gamma), np.cos(gamma), 0],
        [0, 0, 1]
    ])

    # 组合旋转矩阵
    rotation_matrix = Rz @ Ry @ Rx

    # 构造齐次变换矩阵 (相机到仿真坐标系的外参)
    transformation_camera_to_sim = np.eye(4)
    transformation_camera_to_sim[:3, :3] = rotation_matrix
    transformation_camera_to_sim[:3, 3] = [tx, ty, tz]

    return transformation_camera_to_sim


def generate_colored_point_cloud(color_image, depth_image, intrinsic_matrix):
    # 获取图像的尺寸，深度图是二维的 (height: 高度, width: 宽度)
    height, width = depth_image.shape

    # 创建一个空的 Open3D 点云对象
    point_cloud = o3d.geometry.PointCloud()

    # 从内参矩阵中提取相机的焦距和光心位置 (fx, fy 是焦距, cx, cy 是光心)
    fx, fy = intrinsic_matrix[0, 0], intrinsic_matrix[1, 1]
    cx, cy = intrinsic_matrix[0, 2], intrinsic_matrix[1, 2]

    # 将 BGR 彩色图像转换为 RGB 格式（因为 Open3D 需要 RGB 格式）
    color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

    # 初始化用于存储点的 3D 坐标和颜色的列表
    points = []
    colors = []

    # 遍历每个像素 (v: 行, u: 列)
    for v in range(height):
        for u in range(width):
            # 获取深度值，深度值通常是以毫米为单位，乘以 0.001 转换为米
            depth = depth_image[v, u] * 0.001
            if depth > 0:  # 如果深度值大于 0，则计算 3D 坐标
                # 计算 3D 坐标 (X, Y, Z)
                z = depth  # Z 轴为深度值
                x = (u - cx) * z / fx  # X 轴由像素坐标和深度计算得出
                y = (v - cy) * z / fy  # Y 轴由像素坐标和深度计算得出
                points.append([x, y, z])  # 将计算出的 3D 点加入点列表

                # 获取对应点的颜色值（已经转换为 RGB 格式）
                color = color_image[v, u] / 255.0  # 将颜色值归一化到 [0, 1] 范围
                colors.append(color)  # 将颜色加入颜色列表

    # 将计算出的 3D 点赋值给点云对象
    point_cloud.points = o3d.utility.Vector3dVector(np.array(points))
    # 将计算出的颜色赋值给点云对象
    point_cloud.colors = o3d.utility.Vector3dVector(np.array(colors))

    return point_cloud  # 返回生成的点云对象


def transform_point_cloud(point_cloud, transformation_matrix):
    """
    使用变换矩阵将点云转换到仿真坐标系。
    """
    points = np.asarray(point_cloud.points)  # 获取点云的所有点
    # 在点的坐标上应用变换矩阵（齐次坐标）
    ones = np.ones((points.shape[0], 1))  # 添加一个列向量1，用于齐次坐标变换
    points_homogeneous = np.hstack([points, ones])  # 合并坐标和1，形成齐次坐标

    # 应用变换矩阵
    transformed_points = points_homogeneous.dot(transformation_matrix.T)[:, :3]  # 去掉最后一列
    transformed_point_cloud = point_cloud
    transformed_point_cloud.points = o3d.utility.Vector3dVector(transformed_points)  # 更新点云坐标

    return transformed_point_cloud


def crop_point_cloud(point_cloud, x_range, y_range, z_range):
    """
    裁剪点云，只保留在指定范围内的点。
    """
    # 使用Open3D的选择功能，根据范围条件筛选点
    points = np.asarray(point_cloud.points)
    indices = np.where(
        (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1]) &
        (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1]) &
        (points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])
    )[0]

    # 使用筛选后的索引生成新的点云
    cropped_point_cloud = point_cloud.select_by_index(indices)
    return cropped_point_cloud


# 初始化 RealSense 管道对象
pipeline = rs.pipeline()
config = rs.config()

# 配置彩色和深度流，1280x720 分辨率，30 帧率
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)  # 启用深度流
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)  # 启用彩色流

# 启动 RealSense 管道并开始数据流
pipeline.start(config)

# 设置对齐器，用于将深度图像与彩色图像对齐
align = rs.align(rs.stream.color)

try:
    while True:
        # 获取相机的帧数据（深度图像和彩色图像）
        frames = pipeline.wait_for_frames()

        # 对齐深度图像到彩色图像
        aligned_frames = align.process(frames)

        # 获取对齐后的彩色帧和深度帧
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        # 检查是否成功获取到彩色帧和深度帧
        if not color_frame or not depth_frame:
            continue  # 如果没有成功获取，则跳过这一帧

        # 将彩色帧转换为 NumPy 数组，用于后续处理和显示
        color_image = np.asanyarray(color_frame.get_data())

        # 在窗口中显示彩色图像
        cv2.imshow('Realsense', color_image)

        # 检查是否按下键盘输入
        key = cv2.waitKey(1)

        # 如果按下了 Enter 键 (键码为 13)，则生成点云
        if key == 13:
            # 将深度帧转换为 NumPy 数组
            depth_image = np.asanyarray(depth_frame.get_data())

            # 获取相机的内参，用于将像素坐标转换为相机坐标系
            depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            intrinsic_matrix = np.array([[depth_intrinsics.fx, 0, depth_intrinsics.ppx],
                                         [0, depth_intrinsics.fy, depth_intrinsics.ppy],
                                         [0, 0, 1]])

            # 调用函数生成带有颜色信息的 3D 点云
            point_cloud = generate_colored_point_cloud(color_image, depth_image, intrinsic_matrix)

            # # 设定旋转和平移参数
            # alpha, beta, gamma = 27.5, -180.0, 180.0
            # tx, ty, tz = 0.42, -0.77, 1.23
            # # 计算外参矩阵
            # transformation_camera_to_sim = compute_transformation(alpha, beta, gamma, tx, ty, tz)
            #
            # # 使用transform_point_cloud函数转换点云到仿真坐标系
            # transformed_point_cloud = transform_point_cloud(point_cloud_dataset, transformation_camera_to_sim)

            # # 裁剪点云，只保留仿真平台桌面范围内的点
            # x_range = (-0.14, 1.06)  # x 轴范围
            # y_range = (-0.71, 0.80)  # y 轴范围
            # z_range = (-0.015, 0.60)  # z 轴范围

            # 裁剪点云，只保留仿真平台桌面范围内的点
            x_range = (0.18, 0.44)  # x 轴范围
            y_range = (-0.65, -0.2)  # y 轴范围
            z_range = (1.40, 1.6)  # z 轴范围

            cropped_point_cloud = crop_point_cloud(point_cloud, x_range, y_range, z_range)

            # 保存路径
            save_path = r'../../ABB-11-6/Dataset/colored_point_cloud12030202.ply'
            save_directory = os.path.dirname(save_path)  # 获取保存路径的目录部分

            # 检查并创建目录
            if not os.path.exists(save_directory):
                os.makedirs(save_directory)

            # 将点云保存为 PLY 文件
            o3d.io.write_point_cloud(save_path, cropped_point_cloud)
            print(f"点云已保存为 {save_path}")

            o3d.visualization.draw_geometries([cropped_point_cloud])

            break  # 生成点云后退出循环

finally:
    # 结束管道并释放资源
    pipeline.stop()
    # 关闭所有 OpenCV 窗口
    cv2.destroyAllWindows()
