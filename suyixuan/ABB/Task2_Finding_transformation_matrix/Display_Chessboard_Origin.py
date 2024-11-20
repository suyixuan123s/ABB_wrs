"""
Author: Yixuan Su
Date: 2024/11/19 20:40
File: Display_Chessboard_Origin.py
Description:
"""

import numpy as np
import cv2
import open3d as o3d

# 相机内参矩阵和畸变系数（使用十张图片标定得到的结果）
camera_matrix = np.array([[908.05716124, 0., 640.58062138],
                          [0., 907.14785856, 349.07025268],
                          [0., 0., 1.]])
dist_coeffs = np.array([[0.12338635, -0.09838498, -0.00406485, -0.00240096, -0.49340807]])

# 棋盘格尺寸和每个方格的大小（单位：米）
chessboard_size = (11, 8)
square_size = 0.03  # 每个格子的大小为 0.03 米（30 毫米）

# 读取新拍摄的图像
image_path = 'E:\ABB\AI\Depth-Anything-V2\Inter_Realsence_D345_Datasets\color_image_20241025-170147.jpg'
image = cv2.imread(image_path)

if image is None:
    print(f"无法读取图像，请检查文件路径: {image_path}")
else:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 查找棋盘格角点
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        # 绘制标定板坐标系原点
        origin = tuple(corners[0][0].astype(int))  # 获取第一个角点的像素坐标，并转换为整数
        cv2.circle(image, origin, radius=10, color=(0, 0, 255), thickness=-1)  # 在图像上绘制红色圆点表示原点

        # 准备棋盘格在世界坐标系中的坐标
        obj_points = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
        obj_points[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
        obj_points *= square_size

        # 使用 solvePnP 计算相机到标定板的旋转和平移矩阵
        ret, rvec, tvec = cv2.solvePnP(obj_points, corners, camera_matrix, dist_coeffs)

        if ret:
            # 将旋转向量转换为旋转矩阵
            rotation_matrix, _ = cv2.Rodrigues(rvec)

            # 提取标定板坐标系的 X、Y 轴方向向量
            x_axis = rotation_matrix[:, 0]  # X 轴方向
            y_axis = rotation_matrix[:, 1]  # Y 轴方向

            print("标定板坐标系相对于相机坐标系的朝向信息：")
            print(f"X 轴方向向量: {x_axis}")
            print(f"Y 轴方向向量: {y_axis}")

            # 在图像上绘制 X 和 Y 轴的方向，以便可视化
            origin_2d = tuple(corners[0][0].astype(int))
            x_axis_end = (int(origin_2d[0] + x_axis[0] * 100), int(origin_2d[1] + x_axis[1] * 100))
            y_axis_end = (int(origin_2d[0] + y_axis[0] * 100), int(origin_2d[1] + y_axis[1] * 100))

            cv2.arrowedLine(image, origin_2d, x_axis_end, color=(0, 0, 255), thickness=2, tipLength=0.1)  # 绘制 X 轴，红色
            cv2.arrowedLine(image, origin_2d, y_axis_end, color=(0, 255, 0), thickness=2, tipLength=0.1)  # 绘制 Y 轴，绿色

            # 显示带有原点标注的图像
            cv2.imshow("Chessboard with Origin and Axes", image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            # 加载点云文件并转换到桌面坐标系
            point_cloud_path = 'E:\ABB\AI\Depth-Anything-V2\Point_cloud_files\demo25\colored_point_cloud5.ply'
            point_cloud = o3d.io.read_point_cloud(point_cloud_path)

            # 转换点云坐标
            transformation_matrix = np.eye(4)
            transformation_matrix[:3, :3] = rotation_matrix
            transformation_matrix[:3, 3] = tvec[:, 0]

            point_cloud.transform(transformation_matrix)
            o3d.visualization.draw_geometries([point_cloud], window_name="Transformed Point Cloud")
        else:
            print("未能计算出相机到标定板的转换关系。")
    else:
        print("未能找到棋盘格角点，请确保标定板在视野中且清晰可见。")
