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


# 更新面片的顶点索引
def updateid(idlist, face):
    newid0 = idlist.index(face[0])
    newid1 = idlist.index(face[1])
    newid2 = idlist.index(face[2])
    return [newid0, newid1, newid2]


if __name__ == '__main__':
    # 创建Panda3D世界
    base = wd.World(cam_pos=[2.01557, 0.637317, 1.88133], w=1280, h=720, lookat_pos=[0, 0, 0])

    # 添加坐标框架
    gm.gen_frame().attach_to(base)

    # 获取当前文件路径
    this_dir, this_filename = os.path.split(__file__)

    # 生成icosphere（20面体球）
    icosphere = gm.gen_sphere(radius=0.15, rgba=[0, 0, 1, 0.1], subdivisions=0)
    sample = icosphere.objtrm.vertices

    # 在icosphere的每个顶点生成小绿球
    for pnt in sample:
        gm.gen_sphere(pnt, 0.003, [0, 1, 0, 1]).attach_to(base)

    # 设置icosphere为透明蓝色
    icosphere.set_rgba([0, 1, 1, 0.1])
    icosphere.attach_to(base)

    # 加载3D对象（例如：mug.stl）
    # name = "mug.stl"
    # obj = tw.TrimeshHu("./object_g2/", name, scale=0.001)

    name = "rack_10ml_green.STL"
    obj = tw.TrimeshHu("../Task4_ICP_GOFA5/meshes/", name, scale=0.001)
    # 通过 TrimeshHu 获取 Trimesh 网格
    mesh = obj.outputTrimesh

    # # 创建一个 GeometricModel 并将其添加到场景
    # testmesh = gm.GeometricModel(mesh)
    # testmesh.set_rgba([1, 0, 0, 1])  # 设置颜色为红色
    # testmesh.attach_to(base)  # 将模型添加到场景中
    # # base.run()

    # 在一个顶点处生成大球
    gm.gen_sphere(sample[5], 0.03).attach_to(base)

    # 设置原点（这里使用的是icosphere的第6个顶点）
    # origin = np.array(sample[5])
    origin = np.array([0, 0, 0.5])

    # 计算可视面片
    intersector = trimesh.base.ray.ray_pyembree.RayMeshIntersector(mesh)
    faces = mesh.faces
    vertices = mesh.vertices
    check_list = []
    origin_list = []

    # 计算每个面片中心到原点的射线方向
    for face in faces:
        points = [vertices[face[0]], vertices[face[1]], vertices[face[2]]]
        direction = np.array(hm.centerPoint(points) - origin)
        check_list.append(direction)
        origin_list.append(origin)

    # 获取与射线交点的面片索引
    viewed = intersector.intersects_first(ray_origins=origin_list, ray_directions=check_list)
    viewed = hm.listnorepeat(viewed)

    # 提取可视面片的顶点索引
    viewed_faces = [faces[i] for i in viewed]
    list_viewedvertexid = list(set(np.asarray(viewed_faces).flatten().tolist()))

    viewed_vertices = []
    for item in list_viewedvertexid:
        viewed_vertices.append(vertices[item])

    # 更新面片的顶点索引
    viewed_faces = [updateid(list_viewedvertexid, faces[i]) for i in viewed]

    # 创建可视网格并导出为STL
    viewedmesh = trimesh.Trimesh(vertices=viewed_vertices, faces=viewed_faces)
    viewedmesh.export("temp1.stl")

    # # 创建并显示生成的网格
    # test = gm.GeometricModel("temp.stl")
    # test.set_rgba([1, 0, 0, 1])
    # test.attach_to(base)
    # base.run()

    # 使用Open3D读取网格，计算法线，生成点云
    viewedmesh_o3d = o3d.io.read_triangle_mesh("temp1.stl")
    viewedmesh_o3d.compute_vertex_normals()
    pcd = viewedmesh_o3d.sample_points_poisson_disk(number_of_points=1000)

    # 转换为Panda3D格式并显示点云
    pcd_np = vdda.o3dpcd_to_parray(pcd)
    gm.gen_pointcloud(pcd_np, pntsize=5).attach_to(base)

    # 启动Panda3D渲染循环
    base.run()

    # def update(textNode, count, task):
    #     if textNode[0] is not None:
    #         textNode[0].detachNode()
    #         textNode[1].detachNode()
    #         textNode[2].detachNode()
    #
    #     cam_pos = base.cam.getPos()
    #
    #     # 显示摄像机位置（X、Y、Z坐标）
    #     textNode[0] = OnscreenText(
    #         text=str(cam_pos[0])[0:5],
    #         fg=(1, 0, 0, 1),
    #         pos=(1.0, 0.8),
    #         align=TextNode.ALeft
    #     )
    #     textNode[1] = OnscreenText(
    #         text=str(cam_pos[1])[0:5],
    #         fg=(0, 1, 0, 1),
    #         pos=(1.3, 0.8),
    #         align=TextNode.ALeft
    #     )
    #     textNode[2] = OnscreenText(
    #         text=str(cam_pos[2])[0:5],
    #         fg=(0, 0, 1, 1),
    #         pos=(1.6, 0.8),
    #         align=TextNode.ALeft
    #     )
    #
    #     # 创建一个动态蓝色球体
    #     blue_ball = gm.GeometricModel(gm.gen_sphere(radius=0.05, rgba=[0, 0, 1, 1]))
    #     blue_ball.attach_to(base)  # 将蓝色球体附加到场景中
    #
    #     # 更新蓝色球体的位置，使其跟随摄像机
    #     blue_ball.set_pos(cam_pos)
    #
    #     return task.again
    #
    #
    # # 显示“摄像头视角”文字
    # cam_view_text = OnscreenText(
    #     text="Camera View: ",
    #     fg=(0, 0, 0, 1),
    #     pos=(1.15, 0.9),
    #     align=TextNode.ALeft
    # )
    #
    # # 设置摄像机位置文本节点
    # testNode = [None, None, None]
    # count = [0]
    #
    # # 定时更新摄像机位置
    # taskMgr.doMethodLater(0.01, update, "update_cam_pos", extraArgs=[testNode, count], appendTask=True)
    #
    # # 启动Panda3D渲染循环
    # base.run()
