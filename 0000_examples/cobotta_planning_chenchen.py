if __name__ == '__main__':
    import math
    import numpy as np
    import basis.robot_math as rm
    import robot_sim.robots.gofa5.gofa5_handgripper as cbt
    import robot_con.cobotta.cobotta_x as cbtx
    import motion.probabilistic.rrt_connect as rrtc
    import visualization.panda.world as wd
    import modeling.geometric_model as gm
    import time

    start = time.time()
    base = wd.World(cam_pos=[1, 1, .5], lookat_pos=[0, 0, .2])
    gm.gen_frame().attach_to(base)

    robot_s = cbt.GOFA5()
    robot_s.gen_meshmodel().attach_to(base)
    start_conf = robot_s.get_jnt_values(component_name='arm')
    print("start_radians", start_conf)
    # base.run()
    # tgt_pos = np.array([.25, .2, .15])
    # tgt_rotmat = rm.rotmat_from_axangle([0, 1, 0], math.pi * 2/ 3)
    # jnt_values = robot_s.ik(tgt_pos=tgt_pos, tgt_rotmat=tgt_rotmat)
    # robot_s.gen_meshmodel(rgba=(1, 0, 0, 1)).attach_to(base)
    # base.run()
    jnt_values = start_conf + np.array([-0.5, -0.5, 0.5, 0.5, 0.5, 0.5])
    robot_s.fk(component_name="arm", jnt_values=jnt_values)
    start_conf1 = robot_s.get_gl_tcp("arm")[0]
    print('姿态', start_conf1)
    robot_s.gen_meshmodel(rgba=(1, 0, 0, 1)).attach_to(base)
    # base.run()

    tgt_pos = robot_s.get_gl_tcp("arm")[0]
    tgt_rotmat = robot_s.get_gl_tcp("arm")[1]

    goal_pos = tgt_pos
    goal_rotmat = tgt_rotmat
    goal_jnt_values = robot_s.ik(tgt_pos=goal_pos, tgt_rotmat=goal_rotmat)
    print(goal_jnt_values)
    print('//')
    robot_s.fk(component_name="arm", jnt_values=goal_jnt_values)
    robot_s.gen_meshmodel(rgba=(0, 1, 0, 1)).attach_to(base)

    base.run()

    rrtc_planner = rrtc.RRTConnect(robot_s)
    path = rrtc_planner.plan(component_name="arm",
                             start_conf=start_conf,
                             goal_conf=goal_jnt_values,
                             ext_dist=0.05,
                             max_time=300)
    print(path)
    for pose in path[1:-1]:
        robot_s.fk("arm", pose)
        robot_meshmodel = robot_s.gen_meshmodel()
        robot_meshmodel.attach_to(base)
    base.run()
