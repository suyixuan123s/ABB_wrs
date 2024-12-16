"""
Author: Yixuan Su
Date: 2024/12/10 21:05
File: Camera Calibration_color_and_depth.py
Description: 
"""

import cv2
import numpy as np
import glob

# 标定板参数：棋盘格角点的数量（9x6）
checkerboard_size = (9, 6)

# 世界坐标系中标定板角点的真实坐标 (假设每个棋盘格的大小为 1)
world_points = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
world_points[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)

# 用于存储每张图像的角点
obj_points = []  # 世界坐标系中的 3D 点
img_points_color = []  # 颜色相机坐标系中的 2D 点
img_points_depth = []  # 深度相机坐标系中的 2D 点

# 获取颜色相机和深度相机图像路径
color_images = glob.glob("color_calibration_images/*.jpg")  # 替换为颜色相机标定图像路径
depth_images = glob.glob("depth_calibration_images/*.jpg")  # 替换为深度相机标定图像路径

# 读取图像并检测角点
for color_img_path, depth_img_path in zip(color_images, depth_images):
    color_img = cv2.imread(color_img_path)
    depth_img = cv2.imread(depth_img_path, cv2.IMREAD_UNCHANGED)  # 读取深度图像

    gray_color = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
    gray_depth = cv2.cvtColor(depth_img, cv2.COLOR_BGR2GRAY)

    # 检测棋盘格角点
    ret_color, corners_color = cv2.findChessboardCorners(gray_color, checkerboard_size, None)
    ret_depth, corners_depth = cv2.findChessboardCorners(gray_depth, checkerboard_size, None)

    if ret_color and ret_depth:
        # 保存每个图像的角点
        obj_points.append(world_points)
        img_points_color.append(corners_color)
        img_points_depth.append(corners_depth)

        # 可视化角点（可选）
        cv2.drawChessboardCorners(color_img, checkerboard_size, corners_color, ret_color)
        cv2.drawChessboardCorners(depth_img, checkerboard_size, corners_depth, ret_depth)

        cv2.imshow("Color Chessboard corners", color_img)
        cv2.imshow("Depth Chessboard corners", depth_img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# 内参矩阵和畸变系数（从相机手册或先前的标定中获取）
camera_matrix_color = np.array([[651.66, 0, 643.35], [0, 649.87, 355.26], [0, 0, 1]])  # 颜色相机内参矩阵
distortion_coefficients_color = np.array([-0.0528, 0.0600, 0.00088, 0.00137, -0.01998])  # 颜色相机畸变系数

camera_matrix_depth = np.array([[644.34, 0, 648.93], [0, 644.34, 356.65], [0, 0, 1]])  # 深度相机内参矩阵
distortion_coefficients_depth = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # 深度相机畸变系数

# 标定过程（颜色相机）
_, mtx_color, dist_color, rvecs_color, tvecs_color = cv2.calibrateCamera(obj_points, img_points_color,
                                                                         gray_color.shape[::-1], camera_matrix_color,
                                                                         distortion_coefficients_color)

# 标定过程（深度相机）
_, mtx_depth, dist_depth, rvecs_depth, tvecs_depth = cv2.calibrateCamera(obj_points, img_points_depth,
                                                                         gray_depth.shape[::-1], camera_matrix_depth,
                                                                         distortion_coefficients_depth)

# 假设我们选取第一张图像进行外参计算
rvec_color = rvecs_color[0]  # 颜色相机的旋转向量
tvec_color = tvecs_color[0]  # 颜色相机的平移向量

rvec_depth = rvecs_depth[0]  # 深度相机的旋转向量
tvec_depth = tvecs_depth[0]  # 深度相机的平移向量

# 转换为旋转矩阵
rotation_matrix_color, _ = cv2.Rodrigues(rvec_color)
rotation_matrix_depth, _ = cv2.Rodrigues(rvec_depth)

# 输出外参
print("Color Camera Rotation Matrix: \n", rotation_matrix_color)
print("Color Camera Translation Vector: \n", tvec_color)

print("Depth Camera Rotation Matrix: \n", rotation_matrix_depth)
print("Depth Camera Translation Vector: \n", tvec_depth)

# 这里得到的是每个相机的旋转矩阵和平移向量，表示相机相对于标定板的位置和姿态
