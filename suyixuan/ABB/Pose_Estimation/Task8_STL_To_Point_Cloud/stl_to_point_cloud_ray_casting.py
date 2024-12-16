import trimesh
import open3d as o3d
import numpy as np


def stl_to_point_cloud_ray_casting(stl_path, num_points=1000, save_path="point_cloud_ray_casting.ply"):
    # 加载STL模型
    mesh = trimesh.load_mesh(stl_path)

    # 定义摄像机位置和视角
    view_angles = np.linspace(0, 360, num_points)

    # 存储采样点
    points = []

    # 从不同视角投射光线
    for angle in view_angles:
        # 设置摄像机
        cam = mesh.ray.intersects_all(ray_origins=mesh.centroid,
                                      ray_directions=np.array([np.cos(angle), np.sin(angle), 0]))

        # 获取交点
        intersection_points = cam[0]
        points.extend(intersection_points)

    points = np.array(points)

    # 转换为Open3D点云对象
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    # 可视化点云
    o3d.visualization.draw_geometries([point_cloud])

    # 保存点云
    o3d.io.write_point_cloud(save_path, point_cloud)
    print(f"Point cloud saved to {save_path}")

    return point_cloud


# 使用方法
stl_path = r"/suyixuan/ABB/Pose_Estimation/Task3_ICP_pre-processing/temp.stl"
save_path = r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task8_STL_To_Point_Cloud\Datasets\stl_to_point_cloud_ray_casting.ply"
point_cloud = stl_to_point_cloud_ray_casting(stl_path, 1000, save_path)
