"""
Author: Yixuan Su
Date: 2024/12/03 11:28
File: AABB_Axis-Aligned_Bounding Box_Cropping.py
Description: 
"""
import open3d as o3d

# 加载点云
pcd = o3d.io.read_point_cloud("/path/to/your/point_cloud.ply")

# 查看点云范围（可用来手动设置裁剪边界）
print("Point cloud bounds:")
print("Min bound:", pcd.get_min_bound())
print("Max bound:", pcd.get_max_bound())

# 定义裁剪边界 (根据实际点云范围调整)
min_bound = [x_min, y_min, z_min]
max_bound = [x_max, y_max, z_max]

# 创建 AABB 并裁剪点云
aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
cropped_pcd = pcd.crop(aabb)

# 可视化裁剪结果
o3d.visualization.draw_geometries([cropped_pcd], window_name="Cropped Point Cloud")

# 保存裁剪后的点云
o3d.io.write_point_cloud("cropped_point_cloud.ply", cropped_pcd)
