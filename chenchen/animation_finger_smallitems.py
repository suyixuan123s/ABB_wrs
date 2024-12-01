import math

from direct.task.TaskManagerGlobal import taskMgr

import visualization.panda.world as wd
import modeling.geometric_model as gm
import numpy as np
import robot_sim.end_effectors.handgripper.dexgripper.dexgripper_smallitems as cbt

base = wd.World(cam_pos=[1, 1, 0.5], lookat_pos=[0, 0, .2])
gm.gen_frame().attach_to(base)
gripper = cbt.Handgripper()
# gripper.gen_meshmodel().attach_to(base)

rgt_start_pos = [0, -0.1, 0.175]
rgt_start_rot = [[1, 0, 0],
                 [0, 0, -1],
                 [0, 1, 0]]
rgt_end_pos = [0, -0.1, 0.175]
rgt_end_rot = [[1, 0, 0],
               [0, math.cos(90 / 180 * math.pi), -math.sin(90 / 180 * math.pi)],
               [0, math.sin(90 / 180 * math.pi), math.cos(90 / 180 * math.pi)]]
lft_start_pos = [0, 0.003, 0.165]
lft_start_rot = [[-1, 0, 0],
                 [0, math.cos(60 / 180 * math.pi), math.sin(60 / 180 * math.pi)],
                 [0, math.sin(60 / 180 * math.pi), math.cos(60 / 180 * math.pi)]]
lft_end_pos = [0, 0.003, 0.165]
lft_end_rot = [[-1, 0, 0],
               [0, math.cos(60 / 180 * math.pi), math.sin(60 / 180 * math.pi)],
               [0, math.sin(60 / 180 * math.pi), math.cos(60 / 180 * math.pi)]]
print(rgt_end_rot[0][0])


def get_linear_motion(gripper, start_pos, start_rot, end_pos, end_rot, moveinterval, finger):
    path_pos_step = np.linspace(start_pos, end_pos, moveinterval, endpoint=False)
    path_rot_step = np.linspace(start_rot[1][1], end_rot[1][1], moveinterval, endpoint=False)

    count = 0
    jnt_value = np.array([0, 0.0, 0])
    jnt_values_list = []
    color_list = []
    for i in range(len(path_pos_step)):
        gripper.fk(component_name=finger, jnt_values=jnt_value)
        parameter = math.sqrt(1 - path_rot_step[i] * path_rot_step[i])
        rot = [[start_rot[0][0], start_rot[0][1], start_rot[0][2]],
               [start_rot[1][0], path_rot_step[i], -1 * parameter * start_rot[0][0]],
               [start_rot[2][0], parameter, path_rot_step[i] * start_rot[0][0]]]
        jnt_value = gripper.ik(tgt_pos=path_pos_step[i], tgt_rotmat=rot, seed_jnt_values=jnt_value,
                               component_name=finger)

        jnt_values_list.append(jnt_value)
        color_list.append(count / (moveinterval))
        count += 1
    return jnt_values_list


lft_path = get_linear_motion(gripper, start_pos=lft_start_pos, start_rot=lft_start_rot, end_pos=lft_end_pos,
                             end_rot=lft_end_rot, moveinterval=300, finger='lftfinger')
rgt_path = get_linear_motion(gripper, start_pos=rgt_start_pos, start_rot=rgt_start_rot, end_pos=rgt_end_pos,
                             end_rot=rgt_end_rot, moveinterval=300, finger='rgtfinger')
print(lft_path)
print(rgt_path)
lftcounter = [0]
lftflag = [0]
rgtcounter = [0]
rgtflag = [0]
gripper_mesh = []


def update(rgt_path, rgtcounter, rgtflag, lft_path, lftcounter, lftflag, task, ):
    # box = cm.gen_box([0.1, 0.2, 0.3])

    if len(gripper_mesh) != 0:
        for robot_attached in gripper_mesh:
            robot_attached.detach()
        gripper_mesh.clear()
    gripper.fk('lftfinger', lft_path[lftcounter[0]])
    gripper.fk('rgtfinger', rgt_path[rgtcounter[0]])
    gripper_mesh_model = gripper.gen_meshmodel()
    gripper_mesh_model.attach_to(base)
    gripper_mesh.append(gripper_mesh_model)
    if lftflag[0] == 0:
        lftcounter[0] += 1
        if lftcounter[0] == len(lft_path) - 1:
            lftflag[0] = 1
    else:
        lftcounter[0] = lftcounter[0] - 1
        if lftcounter[0] == 0:
            lftflag[0] = 0
    if rgtflag[0] == 0:
        rgtcounter[0] += 1
        if rgtcounter[0] == len(rgt_path) - 1:
            rgtflag[0] = 1
    else:
        rgtcounter[0] = rgtcounter[0] - 1
        if rgtcounter[0] == 0:
            rgtflag[0] = 0

    return task.again


taskMgr.doMethodLater(0.01, update, "update",
                      extraArgs=[rgt_path, rgtcounter, rgtflag, lft_path, lftcounter, lftflag],
                      appendTask=True)

base.run()
