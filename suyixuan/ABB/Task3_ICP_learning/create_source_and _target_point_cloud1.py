"""
Author: Yixuan Su
Date: 2024/11/19 10:36
File: create_source_and _target_point_cloud1.py
"""

import copy
import open3d as o3d
import numpy as np

# 加载 STL 文件
stl_file = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Task4_ICP_GOFA5\meshes\blood_tube10.STL"
mesh = o3d.io.read_triangle_mesh(stl_file)

# 检查 mesh 是否包含有效的顶点和面
if mesh.is_empty() or len(mesh.vertices) == 0 or len(mesh.triangles) == 0:
    print("STL 文件加载失败或文件无有效几何数据。请确保文件包含有效的三角形网格。")
else:
    # 从网格均匀采样点以生成更加密集的点云
    num_points = 20000  # 设置采样点的数量，可以根据需要增加数量
    source_point_cloud = mesh.sample_points_uniformly(number_of_points=num_points)

    # 设置采样后的点云颜色为红色 (RGB: [0, 0.6, 0])
    red_color = np.ones((len(source_point_cloud.points), 3)) * np.array([0, 0.6, 0])
    source_point_cloud.colors = o3d.utility.Vector3dVector(red_color)

    # 可视化源点云
    o3d.visualization.draw_geometries([source_point_cloud], window_name="Source Point Cloud")

    # 创建一个变换矩阵（旋转 + 平移）
    theta = np.pi / 4  # 旋转角度，45度
    R = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

    t = np.array([1, 0, 0])  # 沿 X 轴平移 1 单位

    # 初始变换矩阵（旋转 + 平移）
    trans_init = np.eye(4)
    trans_init[:3, :3] = R
    trans_init[:3, 3] = t

    # 复制源点云并创建目标点云
    target_point_cloud = copy.deepcopy(source_point_cloud)  # 深拷贝源点云
    target_point_cloud.transform(trans_init)  # 对目标点云进行变换

    # 为目标点云设置不同的颜色，例如红色 (RGB: [1, 0, 0])
    target_color = np.ones((len(target_point_cloud.points), 3)) * np.array([0.6, 0, 0])
    target_point_cloud.colors = o3d.utility.Vector3dVector(target_color)

    # 可视化源点云和变换后的目标点云
    o3d.visualization.draw_geometries([source_point_cloud, target_point_cloud],
                                      window_name="Source and Target Point Clouds")

    # 保存点云到文件
    o3d.io.write_point_cloud("source_point_cloud.ply", source_point_cloud)
    o3d.io.write_point_cloud("target_point_cloud.ply", target_point_cloud)

    # 输出变换矩阵
    print("Initial transformation matrix:")
    print(trans_init)
