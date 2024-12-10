"""
Author: Yixuan Su
Date: 2024/12/03 19:48
File: generate_cuboid_stl.py
Description: 
"""
import open3d as o3d
import numpy as np

# 创建一个长方体
def create_cuboid(length, width, height):
    # 长方体的8个顶点
    vertices = np.array([
        [0, 0, 0],
        [length, 0, 0],
        [length, width, 0],
        [0, width, 0],
        [0, 0, height],
        [length, 0, height],
        [length, width, height],
        [0, width, height]
    ])

    # 长方体的12个三角形面，每个三角形由3个顶点定义
    triangles = np.array([
        [0, 3, 1], [1, 3, 2],  # 底面
        [0, 4, 7], [0, 7, 3],  # 左面
        [4, 5, 6], [4, 6, 7],  # 上面
        [5, 1, 2], [5, 2, 6],  # 右面
        [2, 3, 6], [3, 7, 6],  # 后面
        [0, 1, 5], [0, 5, 4]   # 前面
    ])

    # 创建 open3d 中的三角网格
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(triangles)

    # 法线计算
    mesh.compute_vertex_normals()

    return mesh

# 保存STL文件
def save_stl(mesh, file_path):
    o3d.io.write_triangle_mesh(file_path, mesh)
    print(f"STL file saved at {file_path}")

# 设置长方体的尺寸（单位：厘米）
length = 22  # 长度：22厘米
width = 20.6 # 宽度：20.6厘米
height = 7   # 高度：7厘米

# 创建长方体
cuboid_mesh = create_cuboid(length, width, height)

# 保存为STL文件
stl_file_path = "Object_Mesh/box.stl"  # 保存路径
save_stl(cuboid_mesh, stl_file_path)
