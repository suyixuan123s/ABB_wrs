import numpy as np
import open3d as o3d


def generate_grid_points_on_surface(mesh, grid_size=10, num_layers=10):
    """
    按照 XYZ 坐标系均匀地采样 STL 文件的点云，每层按行按列采样。
    :param mesh: 输入的三角网格
    :param grid_size: 每层的网格大小
    :param num_layers: 层数
    :return: 生成的点云
    """
    # 获取网格的边界框
    bbox = mesh.get_axis_aligned_bounding_box()
    min_bound = np.asarray(bbox.min_bound)
    max_bound = np.asarray(bbox.max_bound)

    # 按 Z 轴将整个空间分成多个层
    z_layers = np.linspace(min_bound[2], max_bound[2], num_layers)

    # 网格点列表
    grid_points = []

    for z in z_layers:
        # 在当前 Z 层上生成均匀的网格点
        x_vals = np.linspace(min_bound[0], max_bound[0], grid_size)
        y_vals = np.linspace(min_bound[1], max_bound[1], grid_size)

        for x in x_vals:
            for y in y_vals:
                grid_points.append([x, y, z])  # 添加网格点(x, y, z)

    grid_points = np.array(grid_points)

    # 获取STL文件的点云，均匀采样
    pcd = mesh.sample_points_uniformly(number_of_points=10000)

    # 构建KD树以加速最近邻搜索
    kdtree = o3d.geometry.KDTreeFlann(pcd)

    # 投影点到表面
    projected_points = []
    for point in grid_points:
        # 使用KDTreeFlann来查找最近的点
        [_, idx, _] = kdtree.search_knn_vector_3d(np.array(point), 1)  # 直接传递三维坐标
        # 获取对应最近邻点的实际坐标
        projected_point = np.asarray(pcd.points)[idx[0]]
        projected_points.append(projected_point)

    # 将投影后的点云转为点云对象
    projected_points = np.array(projected_points)
    pcd_projected = o3d.geometry.PointCloud()
    pcd_projected.points = o3d.utility.Vector3dVector(projected_points)

    return pcd_projected


# 读取STL文件并生成按XYZ均匀排列的点云
stl_file_path = "E:\\ABB-Project\\ABB_wrs\\suyixuan\\ABB\\Pose_Estimation\\Task3_ICP_pre-processing\\temp.stl"
mesh = o3d.io.read_triangle_mesh(stl_file_path)

# 检查STL文件是否加载成功
if mesh.is_empty():
    raise FileNotFoundError(f"无法加载STL文件：{stl_file_path}")

# 生成按XYZ均匀排列的点云
grid_size = 30  # 每层的网格大小，可以调整
num_layers = 30  # 总共分成多少层
grid_pcd = generate_grid_points_on_surface(mesh, grid_size, num_layers)

# 显示点云
o3d.visualization.draw_geometries([grid_pcd], window_name="按XYZ均匀排列的点云")

# 保存点云
o3d.io.write_point_cloud("grid_point_cloud_xyz.ply", grid_pcd)
