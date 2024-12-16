"""
Author: Yixuan Su
Date: 2024/11/19 11:25
File: ICP_initial_guess_test.py

"""
import numpy as np
import open3d as o3d

# 加载点云数据
source = o3d.io.read_point_cloud("source_point_cloud.ply")  # 读取源点云
target = o3d.io.read_point_cloud("target_point_cloud.ply")  # 读取目标点云

# 设置不同的颜色，以便于区分源点云和目标点云
source.paint_uniform_color([1, 0, 0])  # 红色，用于源点云
target.paint_uniform_color([0, 1, 0])  # 绿色，用于目标点云

# 添加一个初始变换，使源点云与目标点云有明显的差异
trans_init = np.array([[5.82907243e-01, -7.07109247e-01, -4.00269483e-01, 1.00016291e+00],
                       [5.82925787e-01, 7.07104315e-01, -4.00251189e-01, -1.64462761e-04],
                       [5.66053595e-01, -1.80858771e-05, 8.24368441e-01, 2.23939945e-06],
                       [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

# 对源点云应用初始变换
source.transform(trans_init)

# 可视化源点云和目标点云
# 如果两个点云完全重合，只能看到一个颜色；否则可以看到两个颜色分别对应不同位置
o3d.visualization.draw_geometries([source, target], window_name="Source and Target Point Clouds",
                                  width=1024, height=768)
