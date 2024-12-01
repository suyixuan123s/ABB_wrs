import math
import numpy as np
import robot_sim.end_effectors.handgripper.dexgripper.dexgripper_grabitems as cbt
import visualization.panda.world as wd
import modeling.geometric_model as gm
import basis.robot_math as rm
import modeling.collision_model as cm


def get_figure(gripper, target_pos):
    # rot_x = np.array([[1,0,0],
    #                       [0,math.cos(math.pi/2),-math.sin(math.pi/2)],
    #                       [0,math.sin(math.pi/2),math.cos(math.pi/2)]])
    # rot_y = np.array([[math.cos(math.pi/2),0,math.sin(math.pi/2)],
    #                       [0,1,0],
    #                       [-math.sin(math.pi/2),0,math.cos(math.pi/2)]])
    # rot_z = np.array([[math.cos(-math.pi/2),-math.sin(-math.pi/2),0],
    #                       [math.sin(-math.pi/2),math.cos(-math.pi/2),0],
    #                       [0,0,1]])
    # rel_rot = rot_x
    # rel_rot = np.dot(rel_rot,rot_z)
    # rel_rot = np.dot(rel_rot,rot_x)

    rot_x = np.array([[1, 0, 0],
                      [0, math.cos(math.pi / 2), -math.sin(math.pi / 2)],
                      [0, math.sin(math.pi / 2), math.cos(math.pi / 2)]])
    rot_y = np.array([[math.cos(math.pi / 2), 0, math.sin(math.pi / 2)],
                      [0, 1, 0],
                      [-math.sin(math.pi / 2), 0, math.cos(math.pi / 2)]])
    rot_z = np.array([[math.cos(-math.pi / 2), -math.sin(-math.pi / 2), 0],
                      [math.sin(-math.pi / 2), math.cos(-math.pi / 2), 0],
                      [0, 0, 1]])
    rel_rot = rot_x
    rel_rot = np.dot(rel_rot, rot_z)

    # rot_x = np.array([[1,0,0],
    #                       [0,math.cos(math.pi/180*63.4349),-math.sin(math.pi/180*63.4349)],
    #                       [0,math.sin(math.pi/180*63.4349),math.cos(math.pi/180*63.4349)]])
    # rot_y = np.array([[math.cos(math.pi/2),0,math.sin(math.pi/2)],
    #                       [0,1,0],
    #                       [-math.sin(math.pi/2),0,math.cos(math.pi/2)]])
    # rot_z = np.array([[math.cos(math.pi/6),-math.sin(math.pi/6),0],
    #                       [math.sin(math.pi/6),math.cos(math.pi/6),0],
    #                       [0,0,1]])
    # rel_rot = rot_z
    # rel_rot = np.dot(rel_rot,rot_x)
    T_goal = rm.homomat_from_posrot(target_pos, rel_rot)
    gripper.fix_to(np.dot(T_goal, T_g_f)[:3, 3], np.dot(T_goal, T_g_f)[:3, :3])
    gripper.gen_meshmodel().attach_to(base)


if __name__ == '__main__':
    base = wd.World(cam_pos=[1, 1, 0.5], lookat_pos=[0, 0, .2])
    # gm.gen_frame(length=0.2).attach_to(base)
    grabitembox = cm.CollisionModel("object/hexagon.stl", )
    grabitembox.set_pos(pos=[0, 0, 0])
    grabitembox.set_rgba([126 / 255, 139 / 255, 146 / 255, 1])
    grabitembox.attach_to(base)

    grabitembox = cm.CollisionModel("object/hexagon_side.stl", )
    grabitembox.set_pos(pos=[0, 0, 0])
    grabitembox.set_rgba([190 / 255, 78 / 255, 32 / 255, 0.2])
    grabitembox.attach_to(base)
    #
    # grabitembox = cm.CollisionModel("object/corner.stl", )
    # grabitembox.set_pos(pos=[0, 0, 0])
    # grabitembox.set_rgba([190/255,78/255,32/255, 0.2])
    # grabitembox.attach_to(base)

    gripper_s = cbt.Handgripper()
    # gripper_s.jaw_width(width=0.06,height=0.22)
    gripper_s.jaw_width(width=0.04, height=0.22)
    f_pos = gripper_s.jaw_center_pos
    f_rot = gripper_s.jaw_center_rotmat
    T_g_f = np.linalg.inv(rm.homomat_from_posrot(f_pos, f_rot))

    # print(f_pos)
    # print(f_rot)

    target_pos = [0, 0., 0.03]
    get_figure(gripper=gripper_s, target_pos=target_pos)
    print(gripper_s.lftfinger.jnts[-1]['gl_posq'])
    print(gripper_s.rgtfinger.jnts[-1]['gl_posq'])
    base.run()
