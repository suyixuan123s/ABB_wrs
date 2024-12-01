import numpy as np
import time
import drivers.devices.dynamixel_sdk.sdk_wrapper as mw
import threading
import math


class Gripperhelper(object):
    def __init__(self, gripper, com, peripheral_baud, real=False, sync=True):
        self.gripper = gripper
        print('gripper helper')
        self.real = real
        self.com = com
        self.peripheral_baud = peripheral_baud
        self.sync = sync
        self.init_real_finger()

        # self.finger_s = None

    def init_real_finger(self):
        if self.real:
            self.finger_r = mw.DynamixelMotor(self.com, baud_rate=self.peripheral_baud,
                                              toggle_group_sync_write=self.sync)
            id_list = [0, 1, 2, 3, 4, 5]
            control_mode = 5
            for i in id_list:
                self.finger_r.set_dxl_op_mode(control_mode, i)
                self.finger_r.enable_dxl_torque(i)
                self.finger_r.get_dxl_pos(i)
                self.finger_r.set_dxl_pro_vel(300, i)
                self.finger_r.set_dxl_current_limit(1100, i)
                self.finger_r.set_dxl_goal_current(1100, 1)
        else:
            print("please set real finger on")

    def get_linear_motion(self, start_pos, start_rot, end_pos, end_rot, moveinterval=10, finger='lftfinger'):
        '''

        Parameters
        ----------
        start_pos
        start_rot
        end_pos
        end_rot
        moveinterval

        Returns
        -------

        '''
        path_pos_step = np.linspace(start_pos, end_pos, moveinterval, endpoint=False)
        path_rot_step = np.linspace(start_rot[1][1], end_rot[1][1], moveinterval, endpoint=False)
        count = 0
        jnt_value = np.array([0.0, 0.0, 0.0])
        jnt_values_list = []
        color_list = []
        for i in range(len(path_pos_step)):
            self.gripper.fk(finger, jnt_values=jnt_value)
            parameter = math.sqrt(1 - path_rot_step[i] * path_rot_step[i])
            rot = [[start_rot[0][0], start_rot[0][1], start_rot[0][2]],
                   [start_rot[1][0], path_rot_step[i], -1 * parameter * start_rot[0][0]],
                   [start_rot[2][0], parameter, path_rot_step[i] * start_rot[0][0]]]
            jnt_value = self.gripper.ik(tgt_pos=path_pos_step[i], tgt_rotmat=rot, seed_jnt_values=jnt_value,
                                        component_name=finger)

            jnt_values_list.append(jnt_value)
            color_list.append(count / (moveinterval))
            count += 1
        return jnt_values_list

    def go_fgr_init(self, finger='lft'):
        if finger == 'lft':
            self.finger_r.set_dxl_goal_pos_sync(tgt_pos_list=[2048, 2048, 2048], dxl_id_list=[0, 1, 2])
            time.sleep(1)
        elif finger == 'rgt':
            self.finger_r.set_dxl_goal_pos_sync(tgt_pos_list=[2048, 2048, 2048], dxl_id_list=[3, 4, 5])
            time.sleep(1)
        elif finger == 'dual':
            self.finger_r.set_dxl_goal_pos_sync(tgt_pos_list=[2048, 2048, 2048, 2048, 2048, 2048],
                                                dxl_id_list=[0, 1, 2, 3, 4, 5])
            time.sleep(1)

    def preparation_grabbing_motor(self):
        self.finger_r.set_dxl_goal_pos(tgt_pos=2048, dxl_id=0)
        time.sleep(0.5)
        self.finger_r.set_dxl_goal_pos_sync(tgt_pos_list=[1638, 2219, 2219], dxl_id_list=[3, 4, 5])
        time.sleep(1)
        self.finger_r.set_dxl_goal_pos_sync(tgt_pos_list=[2560, 3072, 2560], dxl_id_list=[0, 1, 2])

    def preparation_grabbing_motor2(self):
        pos0 = self.finger_r.get_dxl_pos(dxl_id=0)
        pos1 = self.finger_r.get_dxl_pos(dxl_id=1)
        pos2 = self.finger_r.get_dxl_pos(dxl_id=2)
        pos3 = self.finger_r.get_dxl_pos(dxl_id=3)
        pos4 = self.finger_r.get_dxl_pos(dxl_id=4)
        pos5 = self.finger_r.get_dxl_pos(dxl_id=5)
        pos_path1 = np.linspace(pos0, 2048, 200)
        for i in pos_path1:
            self.finger_r.set_dxl_goal_pos(tgt_pos=int(i), dxl_id=0)
        time.sleep(0.5)
        pos_path2 = np.linspace([pos3, pos4, pos5], [1638, 2219, 2219], 600)
        for i in pos_path2:
            self.finger_r.set_dxl_goal_pos_sync(tgt_pos_list=[int(i[0]), int(i[1]), int(i[2])], dxl_id_list=[3, 4, 5])
        time.sleep(1)
        pos_path3 = np.linspace([2048, pos1, pos2], [2560, 3072, 2560], 600)
        for i in pos_path3:
            self.finger_r.set_dxl_goal_pos_sync(tgt_pos_list=[int(i[0]), int(i[1]), int(i[2])], dxl_id_list=[0, 1, 2])

    def dif_grabbing1_motor(self):
        self.finger_r.set_dxl_goal_pos(tgt_pos=2048, dxl_id=0)
        time.sleep(0.5)
        self.finger_r.set_dxl_goal_pos_sync(tgt_pos_list=[1638, 2389, 2389], dxl_id_list=[3, 4, 5])
        time.sleep(1)
        self.finger_r.set_dxl_goal_pos_sync(tgt_pos_list=[2560, 3072, 2560], dxl_id_list=[0, 1, 2])

    def dif_grabbing2_motor(self):
        self.finger_r.set_dxl_goal_pos(tgt_pos=2048, dxl_id=0)
        time.sleep(0.5)
        self.finger_r.set_dxl_goal_pos_sync(tgt_pos_list=[1638, 2304, 2304], dxl_id_list=[3, 4, 5])
        time.sleep(1)
        self.finger_r.set_dxl_goal_pos_sync(tgt_pos_list=[2560, 3072, 2560], dxl_id_list=[0, 1, 2])

    def lf_rad2motor(self, rad):
        '''
        convert the jnt angle from path to motor
        Parameters
        ----------
        rad

        Returns motor angle in encoder
        -------

        '''
        deg = np.rad2deg(rad)
        motor = deg * 4096 / 360
        center = 4096 / 2
        return int(center + motor)

    def lf_finger_conf2motor(self, jnt_list):
        '''
        add jnt_list into motor_list
        Parameters
        ----------
        jnt_list

        Returns motor_list
        -------

        '''
        motor_list = []
        for item in jnt_list:
            motor = self.lf_rad2motor(item)
            motor_list.append(motor)
        return motor_list

    def rg_rad2motor(self, rad):
        '''
        convert the jnt angle from path to motor
        Parameters
        ----------
        rad

        Returns motor angle in encoder
        -------

        '''
        deg = np.rad2deg(rad)
        motor = deg * 4096 / 360
        center = 4096 / 2
        return int(center - motor)

    def rg_finger_conf2motor(self, jnt_list):
        '''
        add jnt_list into motor_list
        Parameters
        ----------
        jnt_list

        Returns motor_list
        -------

        '''
        motor_list = []
        for item in jnt_list:
            motor = self.rg_rad2motor(item)
            motor_list.append(motor)
        return motor_list

    def move_con(self, lf_path, rg_path, finger='lft'):
        if finger == 'lft':
            id_group = [0, 1, 2]
            for item in lf_path:
                pos_list = self.lf_finger_conf2motor(item)
                self.finger_r.set_dxl_goal_pos_sync(tgt_pos_list=pos_list, dxl_id_list=id_group)
                time.sleep(0.02)
        elif finger == 'rgt':
            id_group = [3, 4, 5]
            for item in rg_path:
                pos_list = self.rg_finger_conf2motor(item)
                self.finger_r.set_dxl_goal_pos_sync(tgt_pos_list=pos_list, dxl_id_list=id_group)
                time.sleep(0.02)
        else:
            id_group = [0, 1, 2, 3, 4, 5]
            count = 0
            pos0 = self.finger_r.get_dxl_pos(dxl_id=0)
            pos1 = self.finger_r.get_dxl_pos(dxl_id=1)
            pos2 = self.finger_r.get_dxl_pos(dxl_id=2)
            pos_path3 = np.linspace([pos0, pos1, pos2], [2048, 2048, 2048], 600)
            for i in pos_path3:
                self.finger_r.set_dxl_goal_pos_sync(tgt_pos_list=[int(i[0]), int(i[1]), int(i[2])],
                                                    dxl_id_list=[0, 1, 2])
            time.sleep(1)
            pos0 = self.finger_r.get_dxl_pos(dxl_id=0)
            pos1 = self.finger_r.get_dxl_pos(dxl_id=1)
            pos2 = self.finger_r.get_dxl_pos(dxl_id=2)
            pos3 = self.finger_r.get_dxl_pos(dxl_id=3)
            pos4 = self.finger_r.get_dxl_pos(dxl_id=4)
            pos5 = self.finger_r.get_dxl_pos(dxl_id=5)
            lf_pos_list = self.lf_finger_conf2motor(lf_path[0])
            rg_pos_list = self.rg_finger_conf2motor(rg_path[0])
            pos_path4 = np.linspace([pos0, pos1, pos2, pos3, pos4, pos5],
                                    [lf_pos_list[0], lf_pos_list[1], lf_pos_list[2], rg_pos_list[0], rg_pos_list[1],
                                     rg_pos_list[2]], 600)
            for i in pos_path4:
                self.finger_r.set_dxl_goal_pos_sync(
                    tgt_pos_list=[int(i[0]), int(i[1]), int(i[2]), int(i[3]), int(i[4]), int(i[5])],
                    dxl_id_list=[0, 1, 2, 3, 4, 5])
            for item in lf_path:
                lf_pos_list = self.lf_finger_conf2motor(item)
                rg_pos_list = self.rg_finger_conf2motor(rg_path[count])
                self.finger_r.set_dxl_goal_pos_sync(
                    tgt_pos_list=[lf_pos_list[0], lf_pos_list[1], lf_pos_list[2], rg_pos_list[0], rg_pos_list[1],
                                  rg_pos_list[2]], dxl_id_list=id_group)
                count = count + 1
                time.sleep(0.02)

    def disable_torque(self, id):
        self.finger_r.disable_dxl_torque(dxl_id=id)

    def dual_threading(self):
        self.running = True
        countlist0 = [2000]
        countlist1 = [2000]
        countlist2 = [2000]
        lock = threading.Lock()

        def task1():
            while self.running:
                current_pos0 = self.finger_r.get_dxl_pos(dxl_id=0)
                current_pos1 = self.finger_r.get_dxl_pos(dxl_id=1)
                current_pos2 = self.finger_r.get_dxl_pos(dxl_id=2)
                with lock:
                    countlist0.append(current_pos0)
                    countlist1.append(current_pos1)
                    countlist2.append(current_pos2)
                time.sleep(0.002)

        def task2():
            while self.running:
                with lock:
                    if countlist0:
                        current_pos0 = countlist0[-1]
                        if current_pos0 > 1:
                            self.finger_r.set_dxl_goal_pos(tgt_pos=4096 - current_pos0, dxl_id=3)
                    if countlist1:
                        current_pos1 = countlist1[-1]
                        if current_pos1 > 1:
                            self.finger_r.set_dxl_goal_pos(tgt_pos=4096 - current_pos1, dxl_id=4)
                    if countlist2:
                        current_pos2 = countlist2[-1]
                        if current_pos2 > 1:
                            self.finger_r.set_dxl_goal_pos(tgt_pos=4096 - current_pos2, dxl_id=5)
                time.sleep(0.002)

        thread1 = threading.Thread(target=task1)
        thread2 = threading.Thread(target=task2)

        thread1.start()
        thread2.start()

        thread1.join()
        thread2.join()

    def dual_without_thread(self):
        self.running = True
        while self.running:
            current_pos1 = self.finger_r.get_dxl_pos(dxl_id=0)
            current_pos2 = self.finger_r.get_dxl_pos(dxl_id=1)
            current_pos3 = self.finger_r.get_dxl_pos(dxl_id=2)
            current_poslist = [4096 - current_pos1, 4096 - current_pos2, 4096 - current_pos3]
            self.finger_r.set_dxl_goal_pos_sync(tgt_pos_list=current_poslist, dxl_id_list=[3, 4, 5])
            print(current_poslist)
