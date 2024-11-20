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

def main():
    data_path = r'E:\ABB\AI\Depth-Anything-V2\Point_cloud_files\demo25\colored_point_cloud5.ply'
    base = wd.World(cam_pos=[4, 3, 1], lookat_pos=[0, 0, .0])

    # 加载点云
    pcd_visualizer = VisualizePCD(data_path)

    # 相机到标定板的齐次变换矩阵
    transformation_camera_to_board = np.array([
        [-0.0140064, -0.99954686, 0.02664372, 0.11912372],
        [0.84055471, 0.0026615, 0.54172013, 0.09777955],
        [-0.54154557, 0.02998305, 0.84013654, 1.43874376],
        [0., 0., 0., 1.]
    ])

    # 标定板到桌面的平移偏移
    translation_board_to_desk = np.array([0.52, 0.055, 0.006])  # 单位为米

    # 构造标定板到桌面的齐次变换矩阵
    transformation_board_to_desk = np.eye(4)
    transformation_board_to_desk[:3, 3] = translation_board_to_desk

    # 计算相机到桌面的齐次变换矩阵
    transformation_camera_to_desk = np.dot(transformation_board_to_desk, transformation_camera_to_board)

    # 将点云从相机坐标系转换到桌面坐标系
    pcd_visualizer.transform_pcd(transform=transformation_camera_to_desk[:3, 3], rotmat=transformation_camera_to_desk[:3, :3])

    # 可视化点云
    pcd_visualizer.visualize_pcd(base)

if __name__ == "__main__":
    main()
