import robot_sim.end_effectors.handgripper.dexgripper.dexgripper as cbt
import visualization.panda.world as wd
import modeling.geometric_model as gm
import time
import gripperhelper as gh

if __name__ == '__main__':
    base = wd.World(cam_pos=[1, 1, 0.5], lookat_pos=[0, 0, .2])
    gm.gen_frame().attach_to(base)
    gripper = cbt.Handgripper()
    gripper.gen_meshmodel().attach_to(base)
    tgt_pos = gripper.get_gl_tcp("lftfinger")[0]
    tgt_rotmat = gripper.get_gl_tcp("lftfinger")[1]
    peripheral_baud = 57600
    com = 'COM4'
    ghw = gh.Gripperhelper(gripper, com, peripheral_baud, real=True)
    ghw.go_fgr_init('lft')

    start_conf = gripper.get_jnt_values(component_name='lftfinger')
    tgt_pos = gripper.get_gl_tcp("lftfinger")[0]
    tgt_rotmat = gripper.get_gl_tcp("lftfinger")[1]

    ghw.disable_torque(id=0)
    ghw.disable_torque(id=1)
    ghw.disable_torque(id=2)
    ghw.dual_without_thread()

    base.run()
