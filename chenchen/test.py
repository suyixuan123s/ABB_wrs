import math
import numpy as np
import robot_sim.end_effectors.handgripper.dexgripper.dexgripper_smallitems as cbt
import visualization.panda.world as wd
import modeling.geometric_model as gm
import basis.robot_math as rm
import modeling.collision_model as cm


def get_figure(gripper, target_pos, rot_list, rgba):
    for i in rot_list:
        transition_rot = rm.rotmat_from_axangle([0, 0, 1], np.deg2rad(i))
        transpose_y = np.array([[-1, 0, 0],
                                [0, 1, 0],
                                [0, 0, -1]])
        rel_rot = np.dot(transition_rot, transpose_y)
        T_goal = rm.homomat_from_posrot(target_pos, rel_rot)
        # gm.gen_sphere(pos=target_pos, radius=0.01).attach_to(base)
        # gm.gen_frame(pos = target_pos, rotmat=rel_rot).attach_to(base)
        # print(np.dot(T_goal, T_g_f)[:3,3], np.dot(T_goal, T_g_f)[:3,:3])
        gripper.fix_to(np.dot(T_goal, T_g_f)[:3, 3], np.dot(T_goal, T_g_f)[:3, :3])
        gripper.gen_meshmodel(rgba_lftfinger=rgba, rgba_rgtfinger=rgba, rgba_base=rgba).attach_to(base)


if __name__ == '__main__':
    base = wd.World(cam_pos=[1, 1, 0.5], lookat_pos=[0, 0, .2])
    # gm.gen_frame(length=0.2).attach_to(base)
    table = cm.CollisionModel("object/table.stl", )
    table.set_pos(pos=[0, 0, -0])
    table.set_rgba([1, 1, 1, 1])
    table.attach_to(base)
    nut = cm.CollisionModel("object/Nut.stl", )
    nut.set_pos(pos=[0, 0.3, 0])
    nut.set_rgba([0, 0, 0, 1])
    nut.attach_to(base)
    washers = cm.CollisionModel("object/Washers.stl", )
    washers.set_pos(pos=[0.1, 0.3, 0])
    washers.set_rgba([0, 0, 0, 1])
    washers.attach_to(base)
    semicirclekey = cm.CollisionModel("object/Semicirclekey.stl", )
    semicirclekey.set_pos(pos=[-0.1, 0.3, 0])
    semicirclekey.set_rgba([0, 0, 0, 1])
    semicirclekey.attach_to(base)
    testnut1 = cm.CollisionModel("object/Nut.stl", )
    testnut1.set_pos(pos=[0, -0.3, 0])
    testnut1.set_rgba([0, 0, 0, 1])
    testnut1.attach_to(base)

    gripper_s = cbt.Handgripper()
    gripper_s.preparation_grabbing()
    f_pos, f_rot = gripper_s.get_gl_tcp("rgtfinger")
    T_g_f = np.linalg.inv(rm.homomat_from_posrot(f_pos, f_rot))

    semicirclekey_target_pos = [-0.1, 0.3, 0.002]
    semicirclekey_rot_list = np.linspace(0, 360, 10, endpoint=False)
    get_figure(gripper=gripper_s, target_pos=semicirclekey_target_pos, rot_list=semicirclekey_rot_list,
               rgba=[0, 0.3, 0, 0.3])

    nut_target_pos = [0, -0.3, 0.0032]
    nut_rot_list = np.linspace(0, 360, 1, endpoint=False)
    get_figure(gripper=gripper_s, target_pos=nut_target_pos, rot_list=nut_rot_list, rgba=[0.3, 0.3, 0.3, 0.3])
    dif_gripper_s2 = cbt.Handgripper()
    dif_gripper_s2.dif_grabbing2()
    f_pos, f_rot = dif_gripper_s2.get_gl_tcp("rgtfinger")
    T_g_f = np.linalg.inv(rm.homomat_from_posrot(f_pos, f_rot))
    difnut_target_pos1 = [0, -0.3, 0.0032]
    difnut_rot_list1 = np.linspace(0, 360, 1, endpoint=False)
    get_figure(gripper=dif_gripper_s2, target_pos=difnut_target_pos1, rot_list=difnut_rot_list1,
               rgba=[0.3, 0.3, 0.3, 0.3])
    dif_gripper_s1 = cbt.Handgripper()
    dif_gripper_s1.dif_grabbing1()
    f_pos, f_rot = dif_gripper_s1.get_gl_tcp("rgtfinger")
    T_g_f = np.linalg.inv(rm.homomat_from_posrot(f_pos, f_rot))
    difnut_target_pos1 = [0, -0.3, 0.0032]
    difnut_rot_list1 = np.linspace(0, 360, 1, endpoint=False)
    get_figure(gripper=dif_gripper_s1, target_pos=difnut_target_pos1, rot_list=difnut_rot_list1,
               rgba=[0.3, 0.3, 0.3, 0.3])
    base.run()
