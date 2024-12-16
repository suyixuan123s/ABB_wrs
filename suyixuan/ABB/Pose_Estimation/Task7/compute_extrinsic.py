"""
Author: Yixuan Su
Date: 2024/12/10 17:13
File: compute_extrinsic.py
Description: 
"""
import cv2
import numpy as np
import glob

# 标定板参数
checkerboard_size = (9, 6)  # 棋盘格的交点数量
square_size = 0.30  # 每个小方格的实际大小（单位可以是厘米或毫米）

# 生成标定板在世界坐标系中的3D点（假设棋盘格的每个格子的大小为1）
world_points = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
world_points[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
world_points *= square_size  # 缩放为实际单位

# 用于存储每张图像的角点
obj_points = []  # 3D世界坐标中的点
img_points = []  # 图像坐标中的点

# 获取标定图像路径
images = glob.glob("calibration_images/*.jpg")  # 替换为你的标定图像文件夹路径

# 读取图像并检测角点
for image_path in images:
    img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 检测棋盘格角点
    ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

    if ret:
        obj_points.append(world_points)  # 保存对应的3D点
        img_points.append(corners)  # 保存检测到的2D点

        # 可视化角点（可选）
        cv2.drawChessboardCorners(img, checkerboard_size, corners, ret)
        cv2.imshow("Chessboard corners", img)
        cv2.waitKey(500)

cv2.destroyAllWindows()


# 标定相机，计算相机的内参和外参
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None),


# 输出相机内参和畸变系数
print("Camera matrix (内参矩阵): \n", mtx)
print("Distortion coefficients (畸变系数): \n", dist)

# 假设我们选择第一张图像进行外参计算
rvec = rvecs[0]  # 旋转向量
tvec = tvecs[0]  # 平移向量

# 将旋转向量转换为旋转矩阵
rotation_matrix, _ = cv2.Rodrigues(rvec)

print("Rotation Matrix (旋转矩阵):\n", rotation_matrix)
print("Translation Vector (平移向量):\n", tvec)

# 计算标定板相对于相机的位姿（外参）
# 外参由旋转矩阵和位移向量组成
# 这里的外参描述了标定板相对于相机坐标系的位置和姿态
