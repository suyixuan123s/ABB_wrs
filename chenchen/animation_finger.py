from direct.task.TaskManagerGlobal import taskMgr

import visualization.panda.world as wd
import modeling.geometric_model as gm
import numpy as np
import robot_sim.end_effectors.handgripper.dexgripper.dexgripper as cbt

base = wd.World(cam_pos=[1, 1, 0.5], lookat_pos=[0, 0, .2])
gm.gen_frame().attach_to(base)
gripper = cbt.Handgripper()
# gripper.gen_meshmodel().attach_to(base)

rgt_start_pos = [1.46845590e-17, 3.21189322e-02, 2.21169592e-01]
rgt_start_rot = [[ 1.00000000e+00,  9.44534131e-17 ,-1.45168001e-16],
                        [-1.22464680e-16, -2.07058017e-01, -9.78328665e-01],
                        [-1.22464680e-16,  9.78328665e-01 ,-2.07058017e-01]]
rgt_end_pos =[1.95706069e-17, 9.50514357e-02, 1.98134697e-01]
rgt_end_rot = [[1.00000000e+00, 9.44534131e-17, -1.45168001e-16],
                      [-1.22464680e-16, -2.07058017e-01, -9.78328665e-01],
                      [-1.22464680e-16, 9.78328665e-01, -2.07058017e-01]]
# rgt_start_pos = [-1.43257278e-17, -6.52680425e-04,  2.11168445e-01]
# rgt_start_rot = [[-1.00000000e+00, -1.19810707e-16,  2.53572938e-17],
#  [ 0.00000000e+00,  2.07058017e-01,  9.78328665e-01],
#  [-1.22464680e-16,  9.78328665e-01, -2.07058017e-01]]
# rgt_end_pos = [-1.27295540e-17, -9.50514357e-02,  1.98134697e-01]
# rgt_end_rot = [[-1.00000000e+00, -1.19810707e-16,  2.53572938e-17],
#  [ 0.00000000e+00,  2.07058017e-01,  9.78328665e-01],
#  [-1.22464680e-16,  9.78328665e-01, -2.07058017e-01]]

def get_linear_motion(gripper, start_pos, start_rot, end_pos, end_rot, moveinterval, finger):
    path_pos_step = np.linspace(start_pos, end_pos, moveinterval, endpoint=False)
    path_rot_step = np.linspace(start_rot,  end_rot, moveinterval, endpoint=False)
    count = 0
    jnt_value = np.array([0.0, 0.0, 0.0])
    jnt_values_list=[]
    color_list = []
    for i in range(len(path_pos_step)):
        gripper.fk(component_name=finger, jnt_values=jnt_value)
        jnt_value = gripper.ik(tgt_pos=path_pos_step[i], tgt_rotmat=path_rot_step[i], seed_jnt_values=jnt_value,
                                     component_name=finger)


        jnt_values_list.append(jnt_value)
        color_list.append(count / (moveinterval))
        count += 1
    return jnt_values_list


# lft_path = get_linear_motion(gripper, start_pos=lft_start_pos,start_rot=lft_start_rot, end_pos=lft_end_pos, end_rot=lft_end_rot, moveinterval=300,finger='lftfinger')
rgt_path = get_linear_motion(gripper, start_pos=rgt_start_pos,start_rot=rgt_start_rot, end_pos=rgt_end_pos, end_rot=rgt_end_rot, moveinterval=300,finger='rgtfinger')
# print(lft_path)
print(rgt_path)

lftcounter = [0]
lftflag = [0]
rgtcounter = [0]
rgtflag = [0]
gripper_mesh = []
def update( rgt_path,  rgtcounter,rgtflag,task,):
    # box = cm.gen_box([0.1, 0.2, 0.3])

    if len(gripper_mesh) != 0:
        for robot_attached in gripper_mesh:
            robot_attached.detach()
        gripper_mesh.clear()
    # gripper.fk('lftfinger', lft_path[lftcounter[0]])
    gripper.fk('rgtfinger', rgt_path[rgtcounter[0]])
    gripper_mesh_model = gripper.gen_meshmodel()
    gripper_mesh_model.attach_to(base)
    gripper_mesh.append(gripper_mesh_model)
    # if lftflag[0]  == 0:
    #     lftcounter[0]+=1
    #     if lftcounter[0] == len(lft_path)-1:
    #         lftflag[0] = 1
    # else:
    #     lftcounter[0] = lftcounter[0]-1
    #     if lftcounter[0] == 0:
    #         lftflag[0]  = 0
    if rgtflag[0]  == 0:
        rgtcounter[0]+=1
        if rgtcounter[0] == len(rgt_path)-1:
            rgtflag[0] = 1
    else:
        rgtcounter[0] = rgtcounter[0]-1
        if rgtcounter[0] == 0:
            rgtflag[0]  = 0


    return task.again

taskMgr.doMethodLater(0.01, update, "update",
                      extraArgs=[ rgt_path, rgtcounter,rgtflag],
                      appendTask=True)

base.run()