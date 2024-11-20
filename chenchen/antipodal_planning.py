import math
import numpy as np
from direct.task.TaskManagerGlobal import taskMgr

import robot_sim.end_effectors.handgripper.dexgripper.dexgripper_smallitems as cbt
import visualization.panda.world as wd
import modeling.geometric_model as gm
import basis.robot_math as rm
import modeling.collision_model as cm
import robot_sim.robots.gofa5.gofa5_handgripper as gf5
import random
import motion.probabilistic.rrt_connect as rrtc

def get_figure(gripper,target_pos,rot_list,rgba,gripper_pos_list,gripper_rot_list):
    #手做矩阵变换获取手的pos#
    for i in rot_list:
            transition_rot = rm.rotmat_from_axangle([0,0,1], np.deg2rad(i))
            transpose_y = np.array([[-1,0,0],
                                  [0,1,0],
                                  [0,0,-1]])
            rel_rot=np.dot(transition_rot,transpose_y)
            T_goal = rm.homomat_from_posrot(target_pos, rel_rot)
            gripper.fix_to(np.dot(T_goal, T_g_f)[:3,3], np.dot(T_goal, T_g_f)[:3,:3])
            gripper.gen_meshmodel(rgba_lftfinger = rgba,rgba_rgtfinger=rgba,rgba_base=rgba).attach_to(base)
            gripper_pos_list.append(np.dot(T_goal, T_g_f)[:3,3])
            gripper_rot_list.append(np.dot(T_goal, T_g_f)[:3,:3])


if __name__ == '__main__':

    base = wd.World(cam_pos=[1, 1, 0.5], lookat_pos=[0, 0, .2])
    gm.gen_frame(length=0.2).attach_to(base)
    obj = cm.CollisionModel("object/box.stl",)
    obj.set_pos(pos = [0.7,0.3,0])
    obj.set_rgba([0,0,0,1])
    obj.attach_to(base)
    gripper_pos_list = []
    gripper_rot_list = []
    samples_number = 20
    count = 0
    rgt_list = []
    lft_list = []
    gripper_s = cbt.Handgripper()
    start_rgt_conf = gripper_s.get_jnt_values("rgtfinger")
    start_lft_conf = gripper_s.get_jnt_values("lftfinger")
    while count < samples_number:
        gripper_s = cbt.Handgripper()
        gripper_s.random_grab()
        try_rgt_list = gripper_s.get_jnt_values("rgtfinger")
        try_lft_list = gripper_s.get_jnt_values("lftfinger")
        rgt_list.append(try_rgt_list)
        lft_list.append(try_lft_list)
        f_pos, f_rot = gripper_s.get_gl_tcp("rgtfinger")
        T_g_f = np.linalg.inv(rm.homomat_from_posrot(f_pos, f_rot))
        obj_target_pos = [0, 0, 0.045] + obj.get_pos()
        obj_rot_list = np.linspace(random.random()*360, 360, 1 ,endpoint=False)
        get_figure(gripper=gripper_s, target_pos=obj_target_pos, rot_list=obj_rot_list, rgba=[0, 0.3, 0, 0.1],gripper_pos_list=gripper_pos_list,gripper_rot_list=gripper_rot_list)
        count = count+1
    rbt_s = gf5.GOFA5()
    start_conf = rbt_s.get_jnt_values(component_name='arm')
    # jnt_values = start_conf + np.array([-math.pi/2, 0, 0,0, 0, math.pi/2])
    # rbt_s.fk(component_name="arm", jnt_values=jnt_values)
    # rbt_s.gen_meshmodel(rgba=(1, 0, 0, 1)).attach_to(base)
    # aaaaa =rbt_s.get_gl_tcp('arm')
    # base.run()
    count = 0
    while count < samples_number:
        try:
            jnts = rbt_s.ik("arm", gripper_pos_list[count], gripper_rot_list[count])
            rbt_s.fk("arm", jnt_values = jnts)
            pos, rot = rbt_s.get_gl_tcp("arm")
            if not rbt_s.is_collided():
                # rbt_s.gen_meshmodel().attach_to(base)
                break
        except:
            pass
        count = count+1
    gripper_pos1 = gripper_pos_list[count] + [0,0,0.05]
    goal_jnt_values1 = rbt_s.ik(tgt_pos=gripper_pos1, tgt_rotmat=gripper_rot_list[count])
    gripper_pos2 = gripper_pos_list[count]
    goal_jnt_values2 = rbt_s.ik(tgt_pos=gripper_pos2, tgt_rotmat=gripper_rot_list[count])
    gripper_pos3 = gripper_pos2 +[0,0,0.1]
    goal_jnt_values3 = rbt_s.ik(tgt_pos=gripper_pos3, tgt_rotmat=gripper_rot_list[count])
    rrtc_arm_planner = rrtc.RRTConnect(rbt_s)
    ready_pos = np.array([0.7,0.3,0.3])
    ready_rot = np.array([[-1,0,0],[0,0,-1],[0,-1,0]])
    goal_jnt_values4 = rbt_s.ik(tgt_pos=ready_pos, tgt_rotmat=ready_rot)
    final_pos = np.array([0.7,-0.3,0.3])
    final_rot = np.array([[-1,0,0],[0,0,-1],[0,-1,0]])
    goal_jnt_values5 = rbt_s.ik(tgt_pos=ready_pos, tgt_rotmat=ready_rot)
    arm_path1 = rrtc_arm_planner.plan(component_name="arm",
                             start_conf=start_conf,
                             goal_conf=goal_jnt_values1,
                             ext_dist=0.01,
                             max_time=300)
    arm_path2 = rrtc_arm_planner.plan(component_name="arm",
                             start_conf=goal_jnt_values1,
                             goal_conf=goal_jnt_values2,
                             ext_dist=0.01,
                             max_time=300)
    arm_path3 = rrtc_arm_planner.plan(component_name="arm",
                             start_conf=goal_jnt_values2,
                             goal_conf=goal_jnt_values3,
                             ext_dist=0.01,
                             max_time=300)
    arm_path4 = rrtc_arm_planner.plan(component_name="arm",
                             start_conf=goal_jnt_values3,
                             goal_conf=goal_jnt_values4,
                             ext_dist=0.01,
                             max_time=300)
    arm_path5 = rrtc_arm_planner.plan(component_name="arm",
                             start_conf=goal_jnt_values4,
                             goal_conf=goal_jnt_values5,
                             ext_dist=0.01,
                             max_time=300)
    rgt_path1 = np.linspace(start_rgt_conf, rgt_list[count], 300, endpoint=False)
    rgt_len = len(rgt_path1)
    rgt_path1 = [np.array(sublist) for sublist in rgt_path1]
    lft_path1 = np.linspace(start_lft_conf, lft_list[count], 300, endpoint=False)
    lft_len1 = len(lft_path1)
    lft_path1 = [np.array(sublist) for sublist in lft_path1]
    rgt_path2 = np.linspace(rgt_list[count], [math.pi/4,math.pi / 4,0], 300, endpoint=False)
    rgt_path2 = [np.array(sublist) for sublist in rgt_path2]
    lft_path2 = np.linspace(lft_list[count], [math.pi / 4, 0, 0], 300, endpoint=False)
    lft_path2 = [np.array(sublist) for sublist in lft_path2]
    lft_path = []
    lft_path = lft_path + lft_path1
    lft_path = lft_path + lft_path2
    rgt_path = []
    rgt_path = rgt_path + rgt_path1
    rgt_path = rgt_path + rgt_path2

    arm_path = []
    arm_path = arm_path + arm_path1
    arm_len1 = len(arm_path)
    arm_path = arm_path + arm_path2
    arm_len2 = len(arm_path)
    arm_path = arm_path + arm_path3
    arm_len3 = len(arm_path)
    arm_path = arm_path + arm_path4
    arm_path = arm_path + arm_path5
    rbt_s_mesh = []
    armcounter = [0]
    flag = [0]
    handcounter = [0]
    handflag = [0]
    def arm_update(path, armcounter, flag, rgt_path, lft_path, handcounter, handflag , task):
        if len(rbt_s_mesh) != 0:
            for robot_attached in rbt_s_mesh:
                robot_attached.detach()
            rbt_s_mesh.clear()
        if  handcounter[0]== 0 or handcounter[0] >= lft_len1-1:
            rbt_s.fk('arm', path[armcounter[0]])
            if flag[0] == 0:
                armcounter[0] += 1
                if armcounter[0] == len(path) - 1:
                    flag[0] = 1
        if armcounter[0] > arm_len1-1 and handcounter[0] <=lft_len1-1 or armcounter[0]>=arm_len3-1:
            rbt_s.hnd.fk('lftfinger', lft_path[handcounter[0]])
            rbt_s.hnd.fk('rgtfinger', rgt_path[handcounter[0]])
            if handflag[0] == 0:
                handcounter[0] += 1
                if handcounter[0] == len(lft_path) - 1:
                    handflag[0] = 1
        rbt_s_mesh_model = rbt_s.gen_meshmodel()
        rbt_s_mesh_model.attach_to(base)
        rbt_s_mesh.append(rbt_s_mesh_model)
        print(armcounter)
        return task.again

    taskMgr.doMethodLater(0.03, arm_update, "arm_update",
                          extraArgs=[arm_path, armcounter, flag,rgt_path, lft_path,  handcounter,handflag],
                          appendTask=True)
    base.run()