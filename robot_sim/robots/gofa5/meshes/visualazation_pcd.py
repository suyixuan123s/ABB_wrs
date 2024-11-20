# import numpy as np
# import open3d as o3d
# import vision.depth_camera.pcd_data_adapter as vdda
# import visualization.panda.world as wd
# import modeling.geometric_model as gm
# import basis.robot_math as rm
#
#
# class VisualizePCD:
#     def __init__(self, pcd_data_path):
#         if pcd_data_path.endswith('.txt'):
#             self.pcd_np = np.loadtxt(pcd_data_path)
#             self.pcd = vdda.nparray_to_o3dpcd(self.pcd_np)
#         elif pcd_data_path.endswith(('.pcd', '.ply')):
#             self.pcd = o3d.io.read_point_cloud(pcd_data_path)
#             pcd_np = vdda.o3dpcd_to_parray(self.pcd)
#             centroid = np.mean(pcd_np, axis=0)  # 中心点
#             self.pcd_np = pcd_np - centroid
#             self.pcd_colors = np.asarray(self.pcd.colors)
#         else:
#             print('Wrong data type!')
#
#     def transform_pcd(self, transform, rotmat):
#         if transform is not None:
#             self.pcd_np = self.pcd_np + transform
#             self.pcd = vdda.nparray_to_o3dpcd(self.pcd_np)
#         if rotmat.any():
#             self.pcd_np = self.pcd_np @ rotmat.T
#             self.pcd = vdda.nparray_to_o3dpcd(self.pcd_np)
#
#     def visualize_pcd(self, base, rgbas=None, pntsize=3):
#         if rgbas is None:
#             rgbas = self.pcd_colors if self.pcd_colors is not None else [[1, 0, 0, 1]]
#         gm.gen_frame(length=1, thickness=0.05).attach_to(base)
#         gm.gen_pointcloud(self.pcd_np, rgbas=rgbas, pntsize=pntsize).attach_to(base)
#         base.run()
#
#     def save_pcd(self, save_path):
#         o3d.io.write_point_cloud(save_path, self.pcd)
#         print(f"Saved point cloud to {save_path}")
#
#
# def adjust_scale(pcd1_np, pcd2_np):
#     """ 计算点云1和点云2的包围盒并进行非等比例缩放 """
#     bbox1 = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(pcd1_np))
#     bbox2 = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(pcd2_np))
#
#     size1 = np.array(bbox1.get_extent())
#     size2 = np.array(bbox2.get_extent())
#
#     scale_factors = size1 / size2  # 计算点云1和点云2在x、y、z方向上的缩放比例
#     pcd2_np_scaled = pcd2_np * scale_factors  # 对点云2进行非等比例缩放
#
#     return pcd2_np_scaled
#
#
# def main():
#     data_path1 = r'E:\\ABB\\AI\\Depth-Anything-V2\\Point_cloud_files\\demo24\\colored_point_cloud.ply'
#     data_path2 = r'E:\\ABB\\AI\\Depth-Anything-V2\\metric_depth\\output1\\color_image_20241024-191620.ply'
#
#     base = wd.World(cam_pos=[-16.0066, 7.28522, -11.9873], w=3960, h=2540, lookat_pos=[0, 0, 0])
#
#     # 读取点云1
#     pcd1 = o3d.io.read_point_cloud(data_path1)
#     pcd1_np = vdda.o3dpcd_to_parray(pcd1)
#     centroid1 = np.mean(pcd1_np, axis=0)
#
#     # 读取点云2
#     pcd2 = o3d.io.read_point_cloud(data_path2)
#     pcd2_np = vdda.o3dpcd_to_parray(pcd2)
#
#     # 对点云2进行非等比例缩放，使其与点云1的大小一致
#     pcd2_np_scaled = adjust_scale(pcd1_np, pcd2_np)
#
#     # 平移点云1，使其在点云2的上方
#     bbox1 = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(pcd1_np))
#     bbox2_scaled = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(pcd2_np_scaled))
#
#     translation = [0, 0, bbox2_scaled.get_extent()[2] + bbox1.get_extent()[2]]  # 计算平移量
#     pcd1_np_translated = pcd1_np + translation  # 对点云1进行平移
#
#     # 获取点云颜色信息
#     pcd1_colors = np.asarray(pcd1.colors)
#     pcd2_colors = np.asarray(pcd2.colors)
#
#     a1 = np.concatenate((pcd1_colors, np.ones((len(pcd1_colors), 1))), axis=1)
#     a2 = np.concatenate((pcd2_colors, np.ones((len(pcd2_colors), 1))), axis=1)
#
#     # 生成参考坐标系框架，并将其中心设置为点云1的中心
#     frame = gm.gen_frame(length=1, thickness=0.05, pos=centroid1)  # 在生成时设置位置
#     frame.attach_to(base)
#
#     # 可视化点云1和点云2
#     gm.gen_pointcloud(pcd1_np_translated, rgbas=a1, pntsize=3).attach_to(base)
#     gm.gen_pointcloud(pcd2_np_scaled, rgbas=a2, pntsize=3).attach_to(base)
#
#     base.run()
#
#
# # 执行 main 函数
# if __name__ == "__main__":
#     main()
#


import numpy as np
import open3d as o3d
import vision.depth_camera.pcd_data_adapter as vdda
import visualization.panda.world as wd
import modeling.geometric_model as gm


class VisualizePCD:
    def __init__(self, pcd_data_path):
        if pcd_data_path.endswith('.txt'):
            self.pcd_np = np.loadtxt(pcd_data_path)
            self.pcd = vdda.nparray_to_o3dpcd(self.pcd_np)
        elif pcd_data_path.endswith(('.pcd', '.ply')):
            self.pcd = o3d.io.read_point_cloud(pcd_data_path)

            # 调用 Open3D 的 get_center() 方法计算中心点
            center = self.pcd.get_center()

            # 将 Open3D 点云转换为 NumPy 数组
            pcd_np = vdda.o3dpcd_to_parray(self.pcd)

            # 将点云数据相对于中心点进行平移，使得质心位于原点
            self.pcd_np = pcd_np - center

            # pcd_np = vdda.o3dpcd_to_parray(self.pcd)
            # centroid = np.mean(pcd_np, axis=0)  # 中心点
            # self.pcd_np = pcd_np - centroid


            # 提取点云的颜色信息
            self.pcd_colors = np.asarray(self.pcd.colors)
        else:
            print('Wrong data type!')

    def transform_pcd(self, transform, rotmat):
        if transform is not None:
            self.pcd_np = self.pcd_np + transform
            self.pcd = vdda.nparray_to_o3dpcd(self.pcd_np)
        if rotmat.any():
            self.pcd_np = self.pcd_np @ rotmat.T
            self.pcd = vdda.nparray_to_o3dpcd(self.pcd_np)

    def visualize_pcd(self, base, rgbas=None, pntsize=3):
        if rgbas is None:
            rgbas = self.pcd_colors if self.pcd_colors is not None else [[1, 0, 0, 1]]
        gm.gen_frame(length=1, thickness=0.05).attach_to(base)
        gm.gen_pointcloud(self.pcd_np, rgbas=rgbas, pntsize=pntsize).attach_to(base)
        base.run()

    def save_pcd(self, save_path):
        o3d.io.write_point_cloud(save_path, self.pcd)
        print(f"Saved point cloud to {save_path}")


def align_pcds(pcd1_np, pcd2_np):
    """
    通过平移和缩放使两个点云重叠。
    :param pcd1_np: 点云1的NumPy数据
    :param pcd2_np: 点云2的NumPy数据
    :return: 平移并缩放后的点云2
    """
    # 计算质心
    centroid1 = np.mean(pcd1_np, axis=0)
    centroid2 = np.mean(pcd2_np, axis=0)

    # 平移点云2，使得它的质心与点云1的质心对齐
    pcd2_np_aligned = pcd2_np - centroid2 + centroid1

    # 获取两个点云的包围盒
    bbox1 = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(pcd1_np))
    bbox2 = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(pcd2_np_aligned))

    size1 = np.array(bbox1.get_extent())
    size2 = np.array(bbox2.get_extent())

    # 缩放点云2，使其包围盒尺寸与点云1相同
    scale_factors = size1 / size2
    pcd2_np_scaled = (pcd2_np_aligned - centroid1) * scale_factors + centroid1

    return pcd2_np_scaled


def main():
    data_path1 = r'E:\\ABB\\AI\\Depth-Anything-V2\\Point_cloud_files\\demo24\\colored_point_cloud.ply'
    data_path2 = r'E:\\ABB\\AI\\Depth-Anything-V2\\metric_depth\\output1\\color_image_20241024-191620.ply'

    base = wd.World(cam_pos=[-16.0066, 7.28522, -11.9873], w=3960, h=2540, lookat_pos=[0, 0, 0])

    # 读取点云1
    pcd1 = o3d.io.read_point_cloud(data_path1)
    pcd1_np = vdda.o3dpcd_to_parray(pcd1)

    # 读取点云2
    pcd2 = o3d.io.read_point_cloud(data_path2)
    pcd2_np = vdda.o3dpcd_to_parray(pcd2)

    # 调整点云2，使其与点云1重叠
    pcd2_np_aligned = align_pcds(pcd1_np, pcd2_np)

    # 获取点云颜色信息
    pcd1_colors = np.asarray(pcd1.colors)
    pcd2_colors = np.asarray(pcd2.colors)

    a1 = np.concatenate((pcd1_colors, np.ones((len(pcd1_colors), 1))), axis=1)
    a2 = np.concatenate((pcd2_colors, np.ones((len(pcd2_colors), 1))), axis=1)

    # 生成参考坐标系框架
    gm.gen_frame(length=1, thickness=0.05).attach_to(base)

    # 可视化点云1和点云2（重叠后）
    gm.gen_pointcloud(pcd1_np, rgbas=a1, pntsize=3).attach_to(base)
    gm.gen_pointcloud(pcd2_np_aligned, rgbas=a2, pntsize=3).attach_to(base)

    base.run()


# 执行 main 函数
if __name__ == "__main__":
    main()
