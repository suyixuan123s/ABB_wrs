import os
import math
import numpy as np
import modeling.collision_model as cm
import modeling.model_collection as mc
import robot_sim._kinematics.jlchain as jl
import robot_sim.manipulators.gofa5.gofa5 as rbt
# import robot_sim.end_effectors.gripper.robotiq140.robotiq140 as hnd
import robot_sim.end_effectors.gripper.ag145.ag145 as hnd
# import robot_sim.end_effectors.gripper.dh60.dh60 as hnd
import robot_sim.robots.robot_interface as ri
from panda3d.core import CollisionNode, CollisionBox, Point3
import copy
import robot_sim.manipulators.machinetool.machinetool_gripper as machine
import basis.robot_math as rm



class GOFA5(ri.RobotInterface):

    # GOFA5 类的构造函数初始化了机器人的位置、旋转矩阵、名称以及是否启用碰撞检测。通过调用父类的构造函数来初始化基本属性。
    def __init__(self, pos=np.zeros(3), rotmat=np.eye(3), name="ur5e_conveyorbelt", enable_cc=True):
        super().__init__(pos=pos, rotmat=rotmat, name=name)
        this_dir, this_filename = os.path.split(__file__)


        # 初始化基座

        # 此段代码定义了机器人基座及其碰撞模型。基座采用了 STL 格式的三维模型，并且定义了碰撞检测的基本属性。

        # base plate
        self.base_stand = jl.JLChain(pos=pos,
                                     rotmat=rotmat,
                                     homeconf=np.zeros(0),
                                     name='base_stand')

        self.base_stand.lnks[0]['collision_model'] = cm.CollisionModel(
            os.path.join(this_dir, "meshes", "wholetable.stl"),
            cdprimit_type="box", expand_radius=.005,
            userdefined_cdprimitive_fn=self._base_combined_cdnp)
        self.base_stand.lnks[0]['rgba'] = [.35, .35, .35, 1]
        self.base_stand.reinitialize()

        # 初始化 机械臂和夹爪

        # 定义了机械臂和夹爪，分别使用 GOFA5 机械臂和 AG145 夹爪的类，并设置其位置和旋转矩阵。

        # arm
        arm_homeconf = np.zeros(6)
        self.arm = rbt.GOFA5(pos=pos,
                            rotmat=self.base_stand.jnts[-1]['gl_rotmatq'],
                            homeconf=arm_homeconf,
                            name='arm', enable_cc=False)
        # gripper
        self.hnd = hnd.Ag145(pos=self.arm.jnts[-1]['gl_posq'],
                            rotmat=self.arm.jnts[-1]['gl_rotmatq'],
                            name='hnd', enable_cc=False)

       # 初始化品牌标识

       # 定义了两个品牌标识，并将其位置和旋转矩阵设置为与机械臂的关节位置和旋转矩阵一致。

        self.brand =  cm.CollisionModel(os.path.join(this_dir, "meshes", "logo_01.stl"))
        self.brand.set_rgba([1, 0, 0, 1])
        self.brand.set_pos(self.arm.jnts[2]['gl_posq'])
        self.brand.set_rotmat(self.arm.jnts[2]['gl_rotmatq'])

        self.brand_arm = cm.CollisionModel(os.path.join(this_dir, "meshes", "logo_02.stl"))
        self.brand_arm.set_rgba([1, 0, 0, 1])
        self.brand_arm.set_pos(self.arm.jnts[4]['gl_posq'])
        self.brand_arm.set_rotmat(self.arm.jnts[4]['gl_rotmatq'])

        # 初始化
        # TCP
        # 和碰撞检测

        # 设置了工具中心点（TCP）的位置和旋转矩阵，并初始化了用于存储碰撞检测信息的列表。如果启用了碰撞检测，则调用
        # enable_cc
        # 方法。

        # tool center point
        self.arm.jlc.tcp_jnt_id = -1
        self.arm.jlc.tcp_loc_pos = self.hnd.jaw_center_pos
        self.arm.jlc.tcp_loc_rotmat = self.hnd.jaw_center_rotmat
        # a list of detailed information about objects in hand, see CollisionChecker.add_objinhnd
        self.oih_infos = []
        # collision detection
        if enable_cc:
            self.enable_cc()
        # component map
        self.manipulator_dict['arm'] = self.arm
        self.manipulator_dict['hnd'] = self.arm
        self.hnd_dict['hnd'] = self.hnd
        self.hnd_dict['arm'] = self.hnd

    @staticmethod
    def _base_combined_cdnp(name, radius):
        collision_node = CollisionNode(name)
        collision_primitive_c0 = CollisionBox(Point3(-0.1, 0.0, 0.14 - 0.82),
                                              x=.35 + radius, y=.3 + radius, z=.14 + radius)
        collision_node.addSolid(collision_primitive_c0)
        collision_primitive_c1 = CollisionBox(Point3(0.0, 0.0, -.3),
                                              x=.112 + radius, y=.112 + radius, z=.3 + radius)
        collision_node.addSolid(collision_primitive_c1)
        return collision_node

       # 碰撞检测启用方法

       # 该方法配置了碰撞检测，指定了哪些链节和链节对要进行碰撞检测。

    def enable_cc(self):
        # TODO when pose is changed, oih info goes wrong
        super().enable_cc()
        self.cc.add_cdlnks(self.base_stand, [0])
        #self.cc.add_cdlnks(self.machine.base, [0,1,2,3])
        #self.cc.add_cdlnks(self.machine.fingerhigh, [0])
        self.cc.add_cdlnks(self.arm, [1, 2, 3, 4, 5, 6])
        # self.cc.add_cdlnks(self.hnd.lft, [0, 1])
        # self.cc.add_cdlnks(self.hnd.rgt, [1])
        activelist = [self.base_stand.lnks[0],
                      #self.machine.base.lnks[0],
                      #self.machine.base.lnks[1],
                      #self.machine.base.lnks[2],
                      #self.machine.base.lnks[3],
                      #self.machine.fingerhigh.lnks[0],
                      self.arm.lnks[1],
                      self.arm.lnks[2],
                      self.arm.lnks[3],
                      self.arm.lnks[4],
                      self.arm.lnks[5],
                      self.arm.lnks[6]
                      ]
        self.cc.set_active_cdlnks(activelist)
        fromlist = [self.base_stand.lnks[0],
                    #self.machine.base.lnks[0],
                    #self.machine.base.lnks[1],
                    #self.machine.base.lnks[2],
                    #self.machine.base.lnks[3],
                    #self.machine.fingerhigh.lnks[0],
                    self.arm.lnks[1]]
        intolist = [self.arm.lnks[3],
                    self.arm.lnks[4],
                    self.arm.lnks[5],
                    self.arm.lnks[6]]
        self.cc.set_cdpair(fromlist, intolist)
        for oih_info in self.oih_infos:
            objcm = oih_info['collision_model']
            self.hold(objcm)

    def fix_to(self, pos, rotmat):
        self.pos = pos
        self.rotmat = rotmat
        self.base_stand.fix_to(pos=pos, rotmat=rotmat)
        self.arm.fix_to(pos=self.base_stand.jnts[-1]['gl_posq'], rotmat=self.base_stand.jnts[-1]['gl_rotmatq'])
        self.hnd.fix_to(pos=self.arm.jnts[-1]['gl_posq'], rotmat=self.arm.jnts[-1]['gl_rotmatq'])

        for obj_info in self.oih_infos:
            gl_pos, gl_rotmat = self.arm.cvt_loc_tcp_to_gl(obj_info['rel_pos'], obj_info['rel_rotmat'])
            obj_info['gl_pos'] = gl_pos
            obj_info['gl_rotmat'] = gl_rotmat

    def jaw_center_pos(self):
        return self.machine.jaw_center_pos

    def jaw_center_rot(self):
        return self.machine.jaw_center_rot

    def fk(self, component_name='arm', jnt_values=np.zeros(6)):
        """
        :param jnt_values: 7 or 3+7, 3=agv, 7=arm, 1=grpr; metrics: meter-radian
        :param component_name: 'arm', 'agv', or 'all'
        :return:
        author: weiwei
        date: 20201208toyonaka
        """

        def update_oih(component_name='arm'):
            for obj_info in self.oih_infos:
                gl_pos, gl_rotmat = self.cvt_loc_tcp_to_gl(component_name, obj_info['rel_pos'], obj_info['rel_rotmat'])
                obj_info['gl_pos'] = gl_pos
                obj_info['gl_rotmat'] = gl_rotmat

        def update_component(component_name, jnt_values):
            status = self.manipulator_dict[component_name].fk(jnt_values=jnt_values)
            self.hnd_dict[component_name].fix_to(
                pos=self.manipulator_dict[component_name].jnts[-1]['gl_posq'],
                rotmat=self.manipulator_dict[component_name].jnts[-1]['gl_rotmatq'])
            self.brand.set_pos(self.manipulator_dict[component_name].jnts[2]['gl_posq'])
            self.brand.set_rotmat(self.manipulator_dict[component_name].jnts[2]['gl_rotmatq'])
            self.brand_arm.set_pos(self.manipulator_dict[component_name].jnts[4]['gl_posq'])
            self.brand_arm.set_rotmat(self.manipulator_dict[component_name].jnts[4]['gl_rotmatq'])
            update_oih(component_name=component_name)
            return status

        if component_name in self.manipulator_dict:
            if not isinstance(jnt_values, np.ndarray) or jnt_values.size != 6:
                raise ValueError("An 1x6 npdarray must be specified to move the arm!")
            return update_component(component_name, jnt_values)
        else:
            raise ValueError("The given component name is not supported!")

    def get_jnt_values(self, component_name):
        if component_name in self.manipulator_dict:
            return self.manipulator_dict[component_name].get_jnt_values()
        else:
            raise ValueError("The given component name is not supported!")

    def get_jnt_init(self, component_name):
        if component_name in self.manipulator_dict:
            return self.arm.init_jnts
        else:
            raise ValueError("The given component name is not supported!")

    def rand_conf(self, component_name):
        if component_name in self.manipulator_dict:
            return super().rand_conf(component_name)
        else:
            raise NotImplementedError

    def jaw_to(self,hand_name, jawwidth=0.0):
        self.hnd.jaw_to(jawwidth)

    def hold(self, hnd_name, objcm, jawwidth=None):
        """
        the objcm is added as a part of the robot_s to the cd checker
        :param jawwidth:
        :param objcm:
        :return:
        """
        if hnd_name not in self.hnd_dict:
            raise ValueError("Hand name does not exist!")
        if jawwidth is not None:
            self.hnd_dict[hnd_name].jaw_to(jawwidth)
        rel_pos, rel_rotmat = self.manipulator_dict[hnd_name].cvt_gl_to_loc_tcp(objcm.get_pos(), objcm.get_rotmat())
        intolist = [self.base_stand.lnks[0],
                    self.arm.lnks[1],
                    self.arm.lnks[2],
                    self.arm.lnks[3],
                    self.arm.lnks[4]]
        self.oih_infos.append(self.cc.add_cdobj(objcm, rel_pos, rel_rotmat, intolist))
        return rel_pos, rel_rotmat

    def get_oih_list(self):
        return_list = []
        for obj_info in self.oih_infos:
            objcm = obj_info['collision_model']
            objcm.set_pos(obj_info['gl_pos'])
            objcm.set_rotmat(obj_info['gl_rotmat'])
            return_list.append(objcm)
        return return_list

    def release(self, hnd_name, objcm, jawwidth=None):
        """
        the objcm is added as a part of the robot_s to the cd checker
        :param jawwidth:
        :param objcm:
        :return:
        """
        if hnd_name not in self.hnd_dict:
            raise ValueError("Hand name does not exist!")
        if jawwidth is not None:
            print(jawwidth)
            self.hnd_dict[hnd_name].jaw_to(jawwidth)
        for obj_info in self.oih_infos:
            if obj_info['collision_model'] is objcm:
                self.cc.delete_cdobj(obj_info)
                self.oih_infos.remove(obj_info)
                break

    # 生成模型

    # 这两个方法分别生成机器人的简化模型和网格模型，用于可视化。

    def gen_stickmodel(self,
                       tcp_jnt_id=None,
                       tcp_loc_pos=None,
                       tcp_loc_rotmat=None,
                       toggle_tcpcs=False,
                       toggle_jntscs=False,
                       toggle_connjnt=False,
                       name='xarm7_shuidi_mobile_stickmodel'):
        stickmodel = mc.ModelCollection(name=name)
        self.base_stand.gen_stickmodel(tcp_jnt_id=tcp_jnt_id,
                                       tcp_loc_pos=tcp_loc_pos,
                                       tcp_loc_rotmat=tcp_loc_rotmat,
                                       toggle_tcpcs=False,
                                       toggle_jntscs=toggle_jntscs,
                                       toggle_connjnt=toggle_connjnt).attach_to(stickmodel)
        self.arm.gen_stickmodel(tcp_jnt_id=tcp_jnt_id,
                                tcp_loc_pos=tcp_loc_pos,
                                tcp_loc_rotmat=tcp_loc_rotmat,
                                toggle_tcpcs=toggle_tcpcs,
                                toggle_jntscs=toggle_jntscs,
                                toggle_connjnt=toggle_connjnt).attach_to(stickmodel)
        self.hnd.gen_stickmodel(toggle_tcpcs=False,
                                toggle_jntscs=toggle_jntscs).attach_to(stickmodel)

        return stickmodel

    def gen_meshmodel(self,
                      tcp_jnt_id=None,
                      tcp_loc_pos=None,
                      tcp_loc_rotmat=None,
                      toggle_tcpcs=False,
                      toggle_jntscs=False,
                      rgba=None,
                      is_machine=None,
                      is_robot=True,
                      name='xarm_shuidi_mobile_meshmodel'):
        meshmodel = mc.ModelCollection(name=name)
        if is_robot:
            self.base_stand.gen_meshmodel(tcp_jnt_id=tcp_jnt_id,
                                          tcp_loc_pos=tcp_loc_pos,
                                          tcp_loc_rotmat=tcp_loc_rotmat,
                                          toggle_tcpcs=False,
                                          toggle_jntscs=toggle_jntscs).attach_to(meshmodel)
            self.arm.gen_meshmodel(tcp_jnt_id=tcp_jnt_id,
                                   tcp_loc_pos=tcp_loc_pos,
                                   tcp_loc_rotmat=tcp_loc_rotmat,
                                   toggle_tcpcs=toggle_tcpcs,
                                   toggle_jntscs=toggle_jntscs,
                                   rgba=rgba).attach_to(meshmodel)
            self.hnd.gen_meshmodel(toggle_tcpcs=False,
                                   toggle_jntscs=toggle_jntscs,
                                   rgba=rgba).attach_to(meshmodel)
            brand = copy.deepcopy(self.brand)
            brand.attach_to(meshmodel)
            brand_arm = copy.deepcopy(self.brand_arm)
            brand_arm.attach_to(meshmodel)
        for obj_info in self.oih_infos:
            objcm = obj_info['collision_model']
            objcm.set_pos(obj_info['gl_pos'])
            objcm.set_rotmat(obj_info['gl_rotmat'])
            objcm.copy().attach_to(meshmodel)
        return meshmodel


# 主程序
# 主程序初始化了可视化环境，创建并配置了 GOFA5 机器人
# 执行了一个简单的运动学和碰撞检测示例，并在可视化环境中显示结果。

if __name__ == '__main__':
    import time
    import basis.robot_math as rm
    import visualization.panda.world as wd
    import modeling.geometric_model as gm

    base = wd.World(cam_pos=[4, 3, 1], lookat_pos=[0, 0, .0])

    gm.gen_frame().attach_to(base)
    robot_s = GOFA5(enable_cc=True)
    robot_s.hnd.jaw_to(.06)
    robot_s.gen_meshmodel(toggle_tcpcs=False, toggle_jntscs=False).attach_to(base)
    tgt_pos = np.array([.25, .2, .15])
    tgt_rotmat = rm.rotmat_from_axangle([0, 1, 0], math.pi * 2 / 3)
    # gm.gen_frame(pos=tgt_pos, rotmat=tgt_rotmat).attach_to(base)
    robot_s.show_cdprimit()
    # robot_s.gen_stickmodel().attach_to(base)
    base.run()
    component_name = 'arm'
    jnt_values = robot_s.ik(component_name, tgt_pos, tgt_rotmat)
    robot_s.fk(component_name, jnt_values=jnt_values)
    robot_s_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=False)
    robot_s_meshmodel.attach_to(base)
    # robot_s.show_cdprimit()
    robot_s.gen_stickmodel().attach_to(base)
    tic = time.time()
    result = robot_s.is_collided()
    toc = time.time()
    print(result, toc - tic)
    base.run()
