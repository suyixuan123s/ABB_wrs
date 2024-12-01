import numpy as np
import trimesh
import trimeshwraper as tw
import modeling.geometric_model as gm
import humath as hm
import open3d as o3d
from panda3d.core import TextNode
from direct.gui.OnscreenText import OnscreenText
from direct.task.TaskManagerGlobal import taskMgr
import visualization.panda.world as wd
import vision.depth_camera.pcd_data_adapter as vdda
import os


def update_id(idlist, face):
    """
    更新三角形面的顶点ID。
    根据当前的顶点ID列表，将面中的顶点ID更新为最新的索引值。
    """
    newid0 = idlist.index(face[0])
    newid1 = idlist.index(face[1])
    newid2 = idlist.index(face[2])
    return [newid0, newid1, newid2]


if __name__ == '__main__':
    # 初始化Panda3D世界，设置相机位置与视角
    base = wd.World(cam_pos=[2.01557, 0.637317, 1.88133], w=960, h=540, lookat_pos=[0, 0, 0])
    gm.gen_frame().attach_to(base)  # 生成并附加一个坐标框

    # 创建一个示例球体并显示其顶点
    icosphere = gm.gen_sphere(radius=0.15, rgba=[0, 0, 1, 0.1], subdivisions=0)
    sample = icosphere.objtrm.vertices  # 获取球体的顶点数据
    for pnt in sample:
        gm.gen_sphere(pnt, 0.003, [0, 1, 0, 1]).attach_to(base)  # 在每个顶点位置生成小球
    icosphere.set_rgba([0, 1, 1, 0.1])  # 设置球体的透明度颜色
    icosphere.attach_to(base)

    # # 加载3D物体模型，可以更改为其他模型文件名
    name = "mug.stl"  # 物体名称（如“airplaneremesh”，“armadillo.stl”）
    # obj_gm = gm.GeometricModel(f"./object_g2/{name}")
    # obj_gm.set_rgba([0, 1, 0, 0.1])  # 设置物体模型颜色为绿色且半透明
    # # obj_gm.attach_to(base)  # 将物体附加到世界中

    # base.run()

    # 使用trimesh处理模型并进行交集运算
    obj = tw.TrimeshHu("./object_g2/", name, scale=0.001)  # 加载模型并缩放
    mesh = obj.outputTrimesh  # 获取trimesh的网格数据
    testmesh = gm.GeometricModel(mesh)  # 创建一个新的几何模型用于测试
    testmesh.set_rgba([1, 0, 0, 1])  # 设置测试网格颜色为红色
    # testmesh.attach_to(base)  # 可以选择将测试网格附加到世界中进行可视化
    # base.run()

    # 在球体上显示一个示例点
    gm.gen_sphere(sample[5], 0.03).attach_to(base)  # 在第6个顶点处生成一个小球
    origin = np.array(sample[5])  # 设置原点位置为球体的第6个顶点
    # base.run()

    # 初始化ray-triangle交点检测器
    intersector = trimesh.base.ray.ray_pyembree.RayMeshIntersector(mesh)
    faces = mesh.faces  # 获取网格的面
    vertices = mesh.vertices  # 获取网格的顶点
    check_list = []  # 存储射线方向
    origin_list = []  # 存储射线起点

    # 为每个面生成射线，并计算交点
    for face in faces:
        points = [vertices[face[0]], vertices[face[1]], vertices[face[2]]]  # 获取三角形面的三个顶点
        direction = np.array(hm.centerPoint(points) - origin)  # 计算射线方向（面重心到原点）
        check_list.append(direction)  # 添加到射线方向列表
        origin_list.append(origin)  # 添加到射线起点列表

    # 使用trimesh进行射线和三角形面的交点检测
    viewed = intersector.intersects_first(ray_origins=origin_list, ray_directions=check_list)
    viewed = hm.listnorepeat(viewed)  # 去重，确保每个面只检测一次

    # 获取被射线看到的面
    viewed_faces = [faces[i] for i in viewed]
    list_viewedvertexid = list(set(np.asarray(viewed_faces).flatten().tolist()))  # 获取被查看的顶点ID

    # 获取被查看的顶点
    viewed_vertices = [vertices[item] for item in list_viewedvertexid]

    # 更新面顶点ID，使其符合新的顺序
    viewed_faces = [update_id(list_viewedvertexid, faces[i]) for i in viewed]

    # 创建一个新的网格，仅包含被射线看到的部分
    viewedmesh = trimesh.Trimesh(vertices=viewed_vertices, faces=viewed_faces)

    # 导出被查看的网格为STL文件
    viewedmesh.export("temp.stl")
    test = gm.GeometricModel("temp.stl")
    test.set_rgba([1, 0, 0, 1])  # 设置网格颜色为红色
    test.attach_to(base)  # 将查看的网格附加到世界中
    # base.run()

    # 使用Open3D进行点云采样
    viewedmesh_o3d = o3d.io.read_triangle_mesh("temp.stl")
    viewedmesh_o3d.compute_vertex_normals()  # 计算顶点法线
    pcd = viewedmesh_o3d.sample_points_poisson_disk(number_of_points=500)  # 使用泊松盘采样获取点云数据

    # 将Open3D点云转换为Panda3D可用的格式并显示
    pcd_np = vdda.o3dpcd_to_parray(pcd)
    gm.gen_pointcloud(pcd_np, pntsize=5).attach_to(base)  # 在世界中显示点云

    base.run()


    # 设置相机视角文本更新任务
    def update(textNode, count, task):
        """ 更新显示的相机位置文本 """
        if textNode[0] is not None:
            textNode[0].detachNode()
            textNode[1].detachNode()
            textNode[2].detachNode()
        cam_pos = base.cam.getPos()  # 获取当前相机的位置
        # 更新X、Y、Z坐标文本
        textNode[0] = OnscreenText(
            text=str(cam_pos[0])[0:5],
            fg=(1, 0, 0, 1),
            pos=(1.0, 0.8),
            align=TextNode.ALeft)
        textNode[1] = OnscreenText(
            text=str(cam_pos[1])[0:5],
            fg=(0, 1, 0, 1),
            pos=(1.3, 0.8),
            align=TextNode.ALeft)
        textNode[2] = OnscreenText(
            text=str(cam_pos[2])[0:5],
            fg=(0, 0, 1, 1),
            pos=(1.6, 0.8),
            align=TextNode.ALeft)
        return task.again


    # 显示相机视角的文本标签
    cam_view_text = OnscreenText(
        text="Camera View: ",
        fg=(0, 0, 0, 1),
        pos=(1.15, 0.9),
        align=TextNode.ALeft)

    testNode = [None, None, None]  # 存储文本节点
    count = [0]  # 更新计数器
    taskMgr.doMethodLater(0.01, update, "addobject", extraArgs=[testNode, count], appendTask=True)  # 定时更新相机位置

    base.run()  # 启动Panda3D的渲染循环
