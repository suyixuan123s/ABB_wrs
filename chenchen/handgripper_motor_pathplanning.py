import math
import numpy as np
import robot_sim.end_effectors.handgripper.dexgripper.dexgripper as cbt
import visualization.panda.world as wd
import modeling.geometric_model as gm
import time
import drivers.devices.dynamixel_sdk.sdk_wrapper as mw
import gripperhelper as gh

if __name__ == '__main__':
    base = wd.World(cam_pos=[1, 1, 0.5], lookat_pos=[0, 0, .2])
    gm.gen_frame().attach_to(base)
    gripper = cbt.Handgripper()
    gripper.gen_meshmodel().attach_to(base)
    tgt_pos = gripper.get_gl_tcp("rgtfinger")[0]
    tgt_rotmat = gripper.get_gl_tcp("rgtfinger")[1]
    peripheral_baud = 57600
    com = 'COM4'
    ghw = gh.Gripperhelper(gripper, com, peripheral_baud, real=True)
    ghw.go_fgr_init('rgt')

    print("hi")
    start_conf = gripper.get_jnt_values(component_name='rgtfinger')
    print("start_radians", start_conf)
    tgt_pos = gripper.get_gl_tcp("rgtfinger")[0]
    tgt_rotmat = gripper.get_gl_tcp("rgtfinger")[1]

    start_pos = [9.60626733e-18, 6.52680425e-04, 2.11168445e-01]
    start_rot = [[1.00000000e+00, 9.44534131e-17, -1.45168001e-16],
                 [-1.22464680e-16, -2.07058017e-01, -9.78328665e-01],
                 [-1.22464680e-16, 9.78328665e-01, -2.07058017e-01]]
    end_pos = [1.95706069e-17, 9.50514357e-02, 1.98134697e-01]
    end_rot = [[1.00000000e+00, 9.44534131e-17, -1.45168001e-16],
               [-1.22464680e-16, -2.07058017e-01, -9.78328665e-01],
               [-1.22464680e-16, 9.78328665e-01, -2.07058017e-01]]

    path = ghw.get_linear_motion(start_pos, start_rot, end_pos, end_rot, moveinterval=30, finger='rgtfinger')
    ghw.move_con(path, 'rgt')
    time.sleep(5)

    base.run()
