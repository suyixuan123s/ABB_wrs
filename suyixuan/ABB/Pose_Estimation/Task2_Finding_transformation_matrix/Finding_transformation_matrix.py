"""
Author: Yixuan Su
Date: 2024/11/19 20:40
File: Finding_transformation_matrix.py
Description:
"""

import numpy as np
import open3d as o3d
import tkinter as tk
import basis.robot_math as rm
import visualization.panda.world as wd
import modeling.geometric_model as gm
# from robot_sim.robots.GOFA55.GOFA5 import GOFA5
from robot_sim.robots.gofa5.gofa5 import GOFA5
from panda3d.core import AmbientLight
from direct.task import Task


class GOFA5Demo:
    def __init__(self, root):
        # 保存 Tkinter 根窗口
        self.root = root

        # 初始化 Panda3D 世界
        self.base = wd.World(cam_pos=[1.5, 1.5, 1.5], lookat_pos=[0, 0, 0])
        print("Panda3D 环境初始化完成")

        # 添加光源和显示坐标系
        self.add_light()
        gm.gen_frame(pos=[0, 0, 0], rotmat=np.eye(3), length=0.2, thickness=0.005).attach_to(self.base)

        # 初始化 gofa5 机器人并添加到 Panda3D 场景中
        self.setup_robot()

        # 用于存储当前显示的点云对象
        self.pointcloud_obj = None

        # 将 Tkinter 的更新任务添加到 Panda3D 的任务管理器中
        self.base.taskMgr.add(self.tk_update_task, "tk_update")

    def tk_update_task(self, task):
        # 更新 Tkinter 窗口事件
        self.root.update()
        return Task.cont

    def setup_robot(self):
        # 初始化 gofa5 机器人
        self.robot = GOFA5(enable_cc=True)
        self.robot.hnd.jaw_to(0.06)
        self.robot.gen_meshmodel(toggle_tcpcs=False, toggle_jntscs=False).attach_to(self.base)
        print("gofa5 机器人初始化完成")

    def add_light(self):
        # 添加环境光源
        ambient_light = AmbientLight("ambient_light")
        ambient_light.set_color((0.5, 0.5, 0.5, 1))
        ambient_node = self.base.render.attach_new_node(ambient_light)
        self.base.render.set_light(ambient_node)
        print("环境光源添加完成")

    def add_point_cloud(self, pcd_path):
        # 使用 Open3D 加载点云文件
        pcd = o3d.io.read_point_cloud(pcd_path)
        self.pcd_np = np.asarray(pcd.points)

        # 过滤无效数据
        self.pcd_np = self.pcd_np[~np.isnan(self.pcd_np).any(axis=1)]
        print(f"点云加载成功，包含 {len(self.pcd_np)} 个有效点")

        # 检查点云是否包含颜色数据
        if pcd.has_colors():
            self.pcd_colors = np.asarray(pcd.colors)
            self.pcd_colors = np.concatenate((self.pcd_colors, np.ones((len(self.pcd_colors), 1))), axis=1)
            print("点云包含颜色数据")
        else:
            self.pcd_colors = np.array([[1, 0, 0, 1]] * len(self.pcd_np))
            print("点云不包含颜色数据，使用默认红色")

        # 显示原始点云（不进行初始旋转和位置调整）
        self.update_point_cloud(0, 0, 0, 0, 0, 0)

    def update_point_cloud(self, alpha, beta, gamma, tx, ty, tz):
        # 旋转和平移应用
        rotation_matrix = rm.rotmat_from_euler(np.pi * alpha / 180, np.pi * beta / 180, np.pi * gamma / 180)
        pcd_np_transformed = (self.pcd_np @ rotation_matrix.T) + np.array([tx, ty, tz])

        # 如果存在旧点云对象，移除它
        if self.pointcloud_obj:
            self.pointcloud_obj.detach()

        # 显示旋转和平移后的点云
        self.pointcloud_obj = gm.gen_pointcloud(pcd_np_transformed, rgbas=self.pcd_colors, pntsize=3)
        self.pointcloud_obj.attach_to(self.base)
        print(f"点云已更新：alpha={alpha}°, beta={beta}°, gamma={gamma}°, tx={tx}, ty={ty}, tz={tz}")

    def run_panda3d(self):
        # 保持 Panda3D 窗口打开
        self.base.run()


class ControlWindow:
    def __init__(self, root, gofa_demo):
        self.gofa_demo = gofa_demo
        self.root = root

        # 创建旋转角度和平移的滑块+输入框组合
        self.create_slider_and_entry("X 轴旋转角度 (alpha) °", -180, 180, 0, "alpha")
        self.create_slider_and_entry("Y 轴旋转角度 (beta) °", -180, 180, 0, "beta")
        self.create_slider_and_entry("Z 轴旋转角度 (gamma) °", -180, 180, 0, "gamma")
        self.create_slider_and_entry("X 轴平移 (tx)", -2.0, 2.0, 0, "tx")
        self.create_slider_and_entry("Y 轴平移 (ty)", -2.0, 2.0, 0, "ty")
        self.create_slider_and_entry("Z 轴平移 (tz)", -2.0, 2.0, 0, "tz")

    def create_slider_and_entry(self, label, from_, to, initial, var_name):
        frame = tk.Frame(self.root)
        frame.pack(pady=5)

        tk.Label(frame, text=label).pack(side=tk.LEFT)

        # 创建滑块
        slider = tk.Scale(frame, from_=from_, to=to, orient=tk.HORIZONTAL, resolution=0.01 if "t" in var_name else 1)
        slider.set(initial)
        slider.pack(side=tk.LEFT)
        setattr(self, f"{var_name}_slider", slider)

        # 创建输入框
        entry = tk.Entry(frame, width=8)
        entry.insert(0, str(initial))
        entry.pack(side=tk.LEFT)
        setattr(self, f"{var_name}_entry", entry)

        # 绑定滑块和输入框的联动
        slider.bind("<Motion>", lambda event, vn=var_name: self.slider_to_entry(vn))
        entry.bind("<Return>", lambda event, vn=var_name: self.entry_to_slider(vn))

    def slider_to_entry(self, var_name):
        # 滑块的值更新到输入框
        slider = getattr(self, f"{var_name}_slider")
        entry = getattr(self, f"{var_name}_entry")
        entry.delete(0, tk.END)
        entry.insert(0, str(slider.get()))
        self.update_point_cloud()

    def entry_to_slider(self, var_name):
        # 输入框的值更新到滑块
        entry = getattr(self, f"{var_name}_entry")
        slider = getattr(self, f"{var_name}_slider")
        try:
            value = float(entry.get())
            slider.set(value)
            self.update_point_cloud()
        except ValueError:
            pass  # 忽略无效输入

    def update_point_cloud(self):
        # 更新点云的旋转和位置
        alpha = float(self.alpha_entry.get())
        beta = float(self.beta_entry.get())
        gamma = float(self.gamma_entry.get())
        tx = float(self.tx_entry.get())
        ty = float(self.ty_entry.get())
        tz = float(self.tz_entry.get())
        self.gofa_demo.update_point_cloud(alpha, beta, gamma, tx, ty, tz)


if __name__ == '__main__':
    # 创建 Tkinter 窗口
    root = tk.Tk()
    root.title("点云旋转和平移调整")

    # 初始化 Panda3D 显示窗口
    gofa_demo = GOFA5Demo(root)
    # 加载点云路径
    pcd_path = r'E:\ABB-Project\ABB_wrs\suyixuan\ABB\depth_anything_v2\Point_cloud_Dataset\color_image_20241211-100240.ply'
    gofa_demo.add_point_cloud(pcd_path)

    # 创建控制窗口
    control_window = ControlWindow(root, gofa_demo)

    # 主线程运行 Panda3D 窗口
    gofa_demo.run_panda3d()

# 点云已更新：alpha=-154.0°, beta=-3.5°, gamma=0.5°, tx=0.47, ty=-0.64, tz=1.41