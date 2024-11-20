import math
import numpy as np
import robot_sim.end_effectors.handgripper.dexgripper.dexgripper_smallitems as cbt
import visualization.panda.world as wd
import modeling.geometric_model as gm
import time
import gripperhelper as gh


if __name__ == '__main__':

    base = wd.World(cam_pos=[1, 1, 0.5], lookat_pos=[0, 0, .2])
    gm.gen_frame().attach_to(base)
    gripper = cbt.Handgripper()
    gripper.gen_meshmodel().attach_to(base)
    tgt_pos = gripper.get_gl_tcp("rgtfinger")[0]
    tgt_rotmat = gripper.get_gl_tcp("rgtfinger")[1]
    peripheral_baud = 57600
    com = 'COM3'
    ghw = gh.Gripperhelper(gripper, com, peripheral_baud, real=True)
    start_conf = gripper.get_jnt_values(component_name='rgtfinger')
    print("start_radians", start_conf)
    # tgt_pos = gripper.get_gl_tcp("rgtfinger")[0]
    # tgt_rotmat = gripper.get_gl_tcp("rgtfinger")[1]
    # ghw.preparation_grabbing_motor2()
    # entercharacters = input("按下空格继续...")
    # while entercharacters != ' ':
    #     entercharacters = input("按下空格继续...")
    # print("继续执行任务...")
    # base.run()



    # 六十度
    # rgt_start_pos = [0, -0.010, 0.207]
    # rgt_start_rot = [[1, 0, 0],
    #                  [0, 0, -1],
    #                  [0, 1, 0]]
    # rgt_end_pos = [0, -0.0073, 0.16]
    # rgt_end_rot = [[1, 0, 0],
    #                [0, math.cos(60 / 180 * math.pi), -math.sin(60 / 180 * math.pi)],
    #                [0, math.sin(60 / 180 * math.pi), math.cos(60 / 180 * math.pi)]]
    # lft_start_pos = [0, -0.0085, 0.16]
    # lft_start_rot = [[-1, 0, 0],
    #                  [0, math.cos(60 / 180 * math.pi), math.sin(60 / 180 * math.pi)],
    #                  [0, math.sin(60 / 180 * math.pi), math.cos(60 / 180 * math.pi)]]
    # lft_end_pos = [0, -0.0101, 0.18]
    # lft_end_rot = [[-1, 0, 0],
    #                [0, math.cos(60 / 180 * math.pi), math.sin(60 / 180 * math.pi)],
    #                [0, math.sin(60 / 180 * math.pi), math.cos(60 / 180 * math.pi)]]


    # 八十度
    # rgt_start_pos = [0, -0.010, 0.207]
    # rgt_start_rot = [[1, 0, 0],
    #                  [0, 0, -1],
    #                  [0, 1, 0]]
    # rgt_end_pos = [0, -0.0073, 0.16]
    # rgt_end_rot = [[1, 0, 0],
    #                [0, math.cos(80 / 180 * math.pi), -math.sin(80 / 180 * math.pi)],
    #                [0, math.sin(80 / 180 * math.pi), math.cos(80 / 180 * math.pi)]]
    # lft_start_pos = [0, -0.010, 0.18]
    # lft_start_rot = [[-1, 0, 0],
    #                  [0, math.cos(80 / 180 * math.pi), math.sin(80 / 180 * math.pi)],
    #                  [0, math.sin(80 / 180 * math.pi), math.cos(80 / 180 * math.pi)]]
    # lft_end_pos = [0, -0.012, 0.20]
    # lft_end_rot = [[-1, 0, 0],
    #                [0, math.cos(80 / 180 * math.pi), math.sin(80 / 180 * math.pi)],
    #                [0, math.sin(80 / 180 * math.pi), math.cos(80 / 180 * math.pi)]]


    # 九十度
    # rgt_start_pos = [0, -0.010, 0.207]
    # rgt_start_rot = [[1, 0, 0],
    #                  [0, 0, -1],
    #                  [0, 1, 0]]
    # rgt_end_pos = [0, -0.0073, 0.16]
    # rgt_end_rot = [[1, 0, 0],
    #                [0, math.cos(80 / 180 * math.pi), -math.sin(80 / 180 * math.pi)],
    #                [0, math.sin(80 / 180 * math.pi), math.cos(80 / 180 * math.pi)]]
    # lft_start_pos = [0, -0.009, 0.19]
    # lft_start_rot = [[-1, 0, 0],
    #                  [0, math.cos(90 / 180 * math.pi), math.sin(90 / 180 * math.pi)],
    #                  [0, math.sin(90 / 180 * math.pi), math.cos(90 / 180 * math.pi)]]
    # lft_end_pos = [0, -0.01, 0.21]
    # lft_end_rot = [[-1, 0, 0],
    #                [0, math.cos(90 / 180 * math.pi), math.sin(90 / 180 * math.pi)],
    #                [0, math.sin(90 / 180 * math.pi), math.cos(90 / 180 * math.pi)]]


    # 一百度
    rgt_start_pos = [0, -0.010, 0.215]
    rgt_start_rot = [[1, 0, 0],
                     [0, 0, -1],
                     [0, 1, 0]]
    rgt_end_pos = [0, -0.010, 0.20]
    rgt_end_rot = [[1, 0, 0],
                   [0, math.cos(90 / 180 * math.pi), -math.sin(90 / 180 * math.pi)],
                   [0, math.sin(90 / 180 * math.pi), math.cos(90 / 180 * math.pi)]]
    lft_start_pos = [0, -0.0085, 0.20]
    lft_start_rot = [[-1, 0, 0],
                     [0, math.cos(100 / 180 * math.pi), math.sin(100 / 180 * math.pi)],
                     [0, math.sin(100 / 180 * math.pi), math.cos(100 / 180 * math.pi)]]
    lft_end_pos = [0, -0.0101, 0.22]
    lft_end_rot = [[-1, 0, 0],
                   [0, math.cos(100 / 180 * math.pi), math.sin(100 / 180 * math.pi)],
                   [0, math.sin(100 / 180 * math.pi), math.cos(100 / 180 * math.pi)]]
    lf_path = ghw.get_linear_motion(lft_start_pos, lft_start_rot, lft_end_pos, lft_end_rot, moveinterval=100, finger='lftfinger')
    rg_path = ghw.get_linear_motion(rgt_start_pos, rgt_start_rot, rgt_end_pos, rgt_end_rot, moveinterval=300, finger='rgtfinger')
    ghw.move_con(lf_path = lf_path,rg_path=rg_path, finger='lft')
    ghw.disable_torque(id = 0)
    ghw.disable_torque(id = 1)
    ghw.disable_torque(id = 2)
    # base.run()