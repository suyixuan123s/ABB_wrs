"""
Author: Yixuan Su
Date: 2024/11/29 14:54
File: VideoCapture.py
Description: 
"""

import cv2

# 视频文件路径
input_video = 'input_video.mp4'
output_video = 'output_video.mp4'

# 打开视频文件
cap = cv2.VideoCapture(input_video)

# 获取视频的帧率、宽度和高度
fps = cap.get(cv2.CAP_PROP_FPS)
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# 定义裁剪区域，格式为 (start_x, start_y, width, height)
crop_x, crop_y, crop_w, crop_h = 100, 50, 640, 480  # 裁剪为从 (100, 50) 开始，宽高为 640x480

# 创建视频写入对象
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_video, fourcc, fps, (crop_w, crop_h))

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 裁剪每一帧
    cropped_frame = frame[crop_y:crop_y + crop_h, crop_x:crop_x + crop_w]

    # 显示裁剪后的帧（可选）
    cv2.imshow('Cropped Frame', cropped_frame)

    # 写入裁剪后的帧到输出视频
    out.write(cropped_frame)

    # 按 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
out.release()
cv2.destroyAllWindows()
