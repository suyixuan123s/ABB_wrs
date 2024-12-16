"""
Author: Yixuan Su
Date: 2024/12/10 16:57
File: get_realsense_D405_color_and_depth_intrinsics.py
Description: 
"""
import pyrealsense2 as rs

# 初始化管道
pipeline = rs.pipeline()

# 配置流
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)  # 颜色流
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)  # 深度流

# 启动管道
pipeline.start(config)

# 获取内参
frames = pipeline.wait_for_frames()

# 获取颜色相机内参
color_frame = frames.get_color_frame()
color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
print("颜色相机内参：")
print("焦距 (fx, fy):", color_intrinsics.fx, color_intrinsics.fy)
print("主点 (cx, cy):", color_intrinsics.ppx, color_intrinsics.ppy)
print("畸变系数:", color_intrinsics.coeffs)

# 获取深度相机内参
depth_frame = frames.get_depth_frame()
depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
print("深度相机内参：")
print("焦距 (fx, fy):", depth_intrinsics.fx, depth_intrinsics.fy)
print("主点 (cx, cy):", depth_intrinsics.ppx, depth_intrinsics.ppy)
print("畸变系数:", depth_intrinsics.coeffs)

# 停止管道
pipeline.stop()

'''

E:\ABB-Project\ABB_wrs\venv\Scripts\python.exe E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task7\neicanhuode.py 
颜色相机内参：
焦距 (fx, fy): 434.4398193359375 433.24884033203125
主点 (cx, cy): 322.2346496582031 236.84153747558594
畸变系数: [-0.05277087166905403, 0.06000206619501114, 0.000878486258443445, 0.0013654250651597977, -0.01997724175453186]
深度相机内参：
焦距 (fx, fy): 385.1710205078125 385.1710205078125
主点 (cx, cy): 325.3376770019531 237.9300079345703
畸变系数: [0.0, 0.0, 0.0, 0.0, 0.0]

Process finished with exit code 0


'''



'''
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)  # 颜色流
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)  # 深度流

E:\ABB-Project\ABB_wrs\venv\Scripts\python.exe E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task7\neicanhuode.py 
颜色相机内参：
焦距 (fx, fy): 651.65966796875 649.873291015625
主点 (cx, cy): 643.3519897460938 355.2622985839844
畸变系数: [-0.05277087166905403, 0.06000206619501114, 0.000878486258443445, 0.0013654250651597977, -0.01997724175453186]
深度相机内参：
焦距 (fx, fy): 644.3440551757812 644.3440551757812
主点 (cx, cy): 648.9293212890625 356.65423583984375
畸变系数: [0.0, 0.0, 0.0, 0.0, 0.0]

Process finished with exit code 0

'''