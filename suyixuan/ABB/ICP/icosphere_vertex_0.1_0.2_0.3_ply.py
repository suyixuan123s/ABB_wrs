"""
Author: Yixuan Su
Date: 2024/12/02 14:37
File: icosphere_vertex_0.1_0.2_0.3_ply.py
Description: 
"""

import numpy as np
import trimesh
import trimeshwraper as tw
import modeling.geometric_model as gm
import humath as hm
import open3d as o3d
import os
import vision.depth_camera.pcd_data_adapter as vdda
import visualization.panda.world as wd


# 更新面片的顶点索引
def updateid(idlist, face):
    newid0 = idlist.index(face[0])
    newid1 = idlist.index(face[1])
    newid2 = idlist.index(face[2])
    return [newid0, newid1, newid2]

def save_point_cloud_at_vertex(vertex, mesh, output_dir):
    """
    从顶点观察网格并保存点云。
    :param vertex: 观察顶点位置
    :param mesh: 目标网格
    :param output_dir: 输出目录
    """
    # 创建射线与面片的交点
    intersector = trimesh.base.ray.ray_pyembree.RayMeshIntersector(mesh)
    faces = mesh.faces
    vertices = mesh.vertices
    check_list = []
    origin_list = []

    # 计算每个面片中心到原点的射线方向
    for face in faces:
        points = [vertices[face[0]], vertices[face[1]], vertices[face[2]]]
        direction = np.array(hm.centerPoint(points) - vertex)
        check_list.append(direction)
        origin_list.append(vertex)

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

    # 创建可视网格
    viewedmesh = trimesh.Trimesh(vertices=viewed_vertices, faces=viewed_faces)

    # 保存为ply文件
    filename = f"{vertex[0]}_{vertex[1]}_{vertex[2]}.ply"
    filepath = os.path.join(output_dir, filename)
    viewedmesh.export(filepath)

if __name__ == '__main__':
    # 创建输出目录
    output_dir = './point_clouds'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # 创建Panda3D世界
    base = wd.World(cam_pos=[2.01557, 0.637317, 1.88133], w=1280, h=720, lookat_pos=[0, 0, 0])

    # 添加坐标框架
    gm.gen_frame().attach_to(base)

    # 获取当前文件路径
    this_dir, this_filename = os.path.split(__file__)

    # 生成icosphere（20面体球）
    icosphere = gm.gen_sphere(radius=0.15, rgba=[0, 0, 1, 0.1], subdivisions=0)
    sample = icosphere.objtrm.vertices

    # 设置icosphere为透明蓝色
    icosphere.set_rgba([0, 1, 1, 0.1])
    icosphere.attach_to(base)

    # 加载3D对象（例如：rack_10ml_green.STL）
    name = "rack_10ml_green.STL"
    obj = tw.TrimeshHu("../Task4_ICP_GOFA5/meshes/", name, scale=0.001)
    # 通过 TrimeshHu 获取 Trimesh 网格
    mesh = obj.outputTrimesh

    # 提取icosphere的前三个顶点的点云并保存为文件
    for vertex in sample[:3]:  # 仅提取前三个顶点
        save_point_cloud_at_vertex(vertex, mesh, output_dir)

    # 启动Panda3D渲染循环
    base.run()
