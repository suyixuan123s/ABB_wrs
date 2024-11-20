import os
import math
import numpy as np
import basis.robot_math as rm
import modeling.model_collection as mc
import modeling.collision_model as cm
import robot_sim._kinematics.jlchain as jl
from panda3d.core import CollisionNode, CollisionBox, Point3
import robot_sim.end_effectors.handgripper.dexgripper_interface as gf
import robot_sim.end_effectors.handgripper.finger.leftfinger.leftfinger as lf
import robot_sim.end_effectors.handgripper.finger.finger_4_nails.fingernails as fin


class Handgripper(gf.DexGripperInterface):

    def __init__(self, pos=np.zeros(3), rotmat=np.eye(3), name='hand_griper', enable_cc=True):
        super().__init__(pos=pos, rotmat=rotmat, name=name)
        this_dir, this_filename = os.path.split(__file__)
        # base
        self.base = jl.JLChain(pos=pos, rotmat=rotmat, homeconf=np.zeros(0), name='base')
        self.base.lnks[0]['name'] = "hand_gripper_base"
        self.base.lnks[0]['loc_pos'] = np.array([0, 0, 0])
        self.base.lnks[0]['loc_rotmat'] = rm.rotmat_from_euler(math.pi / 2.0*0, 0,
                                                                   math.pi / 2.0*0)  # left from robot_s view
        self.base.lnks[0]['collision_model'] = cm.CollisionModel(
            os.path.join(this_dir, "meshes", "1231231221.stl"),
            cdprimit_type="box", expand_radius=.005)
        self.base.lnks[0]['rgba'] = [.3, .3, .3, 1.0]
        self.lft = jl.JLChain(pos=np.zeros(3), rotmat=np.eye(3), homeconf=np.zeros(1), name='lft')
        self.lft.jnts[1]['loc_pos'] = np.array([0,0.03919,0.09419])
        self.lft.jnts[1]['loc_rotmat'] = rm.rotmat_from_euler(math.pi *3/4 , math.pi , math.pi )
        self.rgt = jl.JLChain(pos=np.zeros(3), rotmat=np.eye(3), homeconf=np.zeros(1), name='lft')
        self.rgt.jnts[1]['loc_pos'] = np.array([0,-0.03919,0.09419])
        self.rgt.jnts[1]['loc_rotmat'] = rm.rotmat_from_euler(math.pi *3/4 , math.pi , math.pi*0 )

        self.base.reinitialize()
        self.lft.reinitialize()
        self.rgt.reinitialize()

        self.lftfinger = lf.Leftfinger(pos=pos + np.array([0,0.03919,0.09419]),
                            rotmat=rm.rotmat_from_euler(math.pi *3/4 , math.pi , math.pi ),
                            homeconf=np.zeros(3),
                            name='lftfinger', enable_cc=False)
        self.rgtfinger = fin.Fingernails(pos=pos + np.array([0,-0.00805,0.0624]),
                            rotmat=rm.rotmat_from_euler(math.pi *3/4 , math.pi , math.pi*0 ),
                            homeconf=np.zeros(4),
                            name='rgtfinger', enable_cc=False)
        self.lft_oih_infos = []
        self.rgt_oih_infos = []
        self.jaw_center_pos = np.array([0, 0, .18169])  # position for initial state (fully open)
        self.jaw_center_rotmat = rotmat
        if enable_cc:
            self.enable_cc()
        self.manipulator_dict['rgtfinger'] = self.rgtfinger
        self.manipulator_dict['lftfinger'] = self.lftfinger
        self.jawwidth_rng = [0.0, .05]

    @staticmethod
    def _base_combined_cdnp(name, radius):
        collision_node = CollisionNode(name)
        collision_primitive_c0 = CollisionBox(Point3(0.54, 0.0, 0.39),
                                              x=.54 + radius, y=.6 + radius, z=.39 + radius)
        collision_node.addSolid(collision_primitive_c0)
        collision_primitive_c1 = CollisionBox(Point3(0.06, 0.0, 0.9),
                                              x=.06 + radius, y=.375 + radius, z=.9 + radius)
        collision_node.addSolid(collision_primitive_c1)
        collision_primitive_c2 = CollisionBox(Point3(0.18, 0.0, 1.77),
                                              x=.18 + radius, y=.21 + radius, z=.03 + radius)
        collision_node.addSolid(collision_primitive_c2)
        collision_primitive_l0 = CollisionBox(Point3(0.2425, 0.345, 1.33),
                                              x=.1225 + radius, y=.06 + radius, z=.06 + radius)
        collision_node.addSolid(collision_primitive_l0)
        collision_primitive_r0 = CollisionBox(Point3(0.2425, -0.345, 1.33),
                                              x=.1225 + radius, y=.06 + radius, z=.06 + radius)
        collision_node.addSolid(collision_primitive_r0)
        collision_primitive_l1 = CollisionBox(Point3(0.21, 0.405, 1.07),
                                              x=.03 + radius, y=.06 + radius, z=.29 + radius)
        collision_node.addSolid(collision_primitive_l1)
        collision_primitive_r1 = CollisionBox(Point3(0.21, -0.405, 1.07),
                                              x=.03 + radius, y=.06 + radius, z=.29 + radius)
        collision_node.addSolid(collision_primitive_r1)
        return collision_node

    def enable_cc(self):
        super().enable_cc()
        # raise NotImplementedError
        self.cc.add_cdlnks(self.base, [0])
        self.cc.add_cdlnks(self.lftfinger, [0, 1, 2, 3])
        self.cc.add_cdlnks(self.rgtfinger, [0, 1, 2, 3])

        # lnks used for cd with external stationary objects
        activelist = [self.base.lnks[0],
                      self.lftfinger.lnks[0],
                      self.lftfinger.lnks[1],
                      self.lftfinger.lnks[2],
                      self.lftfinger.lnks[3],

                      self.rgtfinger.lnks[0],
                      self.rgtfinger.lnks[1],
                      self.rgtfinger.lnks[2],
                      self.rgtfinger.lnks[3]]
        self.cc.set_active_cdlnks(activelist)
        fromlist = [self.base.lnks[0]]
        intolist = [self.lftfinger.lnks[1],
                    self.lftfinger.lnks[2],
                    self.lftfinger.lnks[3],
                    self.rgtfinger.lnks[1],
                    self.rgtfinger.lnks[2],
                    self.rgtfinger.lnks[3]]
        self.cc.set_cdpair(fromlist, intolist)
        fromlist = [self.lftfinger.lnks[0]]
        intolist = [self.lftfinger.lnks[2],
                    self.lftfinger.lnks[3],
                    self.rgtfinger.lnks[1],
                    self.rgtfinger.lnks[2],
                    self.rgtfinger.lnks[3]]
        self.cc.set_cdpair(fromlist, intolist)
        fromlist = [self.lftfinger.lnks[1]]
        intolist = [self.lftfinger.lnks[3],
                    self.rgtfinger.lnks[0],
                    self.rgtfinger.lnks[1],
                    self.rgtfinger.lnks[2],
                    self.rgtfinger.lnks[3]]
        self.cc.set_cdpair(fromlist, intolist)
        fromlist = [self.lftfinger.lnks[2]]
        intolist = [self.rgtfinger.lnks[0],
                    self.rgtfinger.lnks[1],
                    self.rgtfinger.lnks[2],
                    self.rgtfinger.lnks[3]]
        self.cc.set_cdpair(fromlist, intolist)
        fromlist = [self.lftfinger.lnks[3]]
        intolist = [self.rgtfinger.lnks[0],
                    self.rgtfinger.lnks[1],
                    self.rgtfinger.lnks[2],
                    self.rgtfinger.lnks[3]]
        self.cc.set_cdpair(fromlist, intolist)
        fromlist = [self.rgtfinger.lnks[0]]
        intolist = [self.rgtfinger.lnks[2],
                    self.rgtfinger.lnks[3]]
        self.cc.set_cdpair(fromlist, intolist)
        fromlist = [self.rgtfinger.lnks[1]]
        intolist = [self.rgtfinger.lnks[3]]
        self.cc.set_cdpair(fromlist, intolist)


    # def fix_to(self, pos, rotmat):
    #     self.pos = pos
    #     self.rotmat = rotmat
    #     self.base.fix_to(self.pos, self.rotmat)
    #     A = rm.rotmat_from_euler(math.pi * 3 / 4, math.pi, math.pi )
    #     B = rm.rotmat_from_euler(math.pi *3/4 , math.pi , math.pi*0 )
    #     C =np.array([0,-0.03919,0.09419])
    #     D = np.dot(C,self.rotmat)
    #     E = np.array([-D[0],-D[1],D[2]])
    #     print(D)
    #     print(E)
    #     F = np.array([0,0.03919,0.09419])
    #     G = np.dot(F,self.rotmat)
    #     H = np.array([-G[0],-G[1],G[2]])
    #     self.lftfinger.fix_to(pos=self.base.jnts[-1]['gl_posq']+E, rotmat=np.dot(self.base.jnts[-1]['gl_rotmatq'],A))
    #     self.rgtfinger.fix_to(pos=self.base.jnts[-1]['gl_posq']+H, rotmat=np.dot(self.base.jnts[-1]['gl_rotmatq'],B))


    def fix_to(self, pos, rotmat):
        self.pos = pos
        self.rotmat = rotmat
        print(self.rotmat)
        self.base.fix_to(self.pos, self.rotmat)
        self.lft.fix_to(self.pos, self.rotmat)
        self.rgt.fix_to(self.pos, self.rotmat)
        self.lftfinger.fix_to(pos=self.lft.jnts[-1]['gl_posq'], rotmat=self.lft.jnts[-1]['gl_rotmatq'])
        self.rgtfinger.fix_to(pos=self.rgt.jnts[-1]['gl_posq'], rotmat=self.rgt.jnts[-1]['gl_rotmatq'])

    def fk(self, component_name, jnt_values):
        """
        :param jnt_values: 1x6 or 1x12 nparray
        :hnd_name 'lftfinger', 'rgtfinger', 'both_arm'
        :param component_name:
        :return:
        author: weiwei
        date: 20201208toyonaka
        """

        def update_oih(component_name='rgtfinger'):
            # inline function for update objects in hand
            if component_name == 'rgtfinger':
                oih_info_list = self.rgt_oih_infos
            elif component_name == 'lftfinger':
                oih_info_list = self.lft_oih_infos
            for obj_info in oih_info_list:
                gl_pos, gl_rotmat = self.cvt_loc_tcp_to_gl(component_name, obj_info['rel_pos'], obj_info['rel_rotmat'])
                obj_info['gl_pos'] = gl_pos
                obj_info['gl_rotmat'] = gl_rotmat

        def update_component(component_name, jnt_values):
            status = self.manipulator_dict[component_name].fk(jnt_values=jnt_values)
            update_oih(component_name=component_name)
            return status

        super().fk(component_name, jnt_values)
        # examine length
        if component_name == 'lftfinger' or component_name == 'rgtfinger':
            if not isinstance(jnt_values, np.ndarray) or jnt_values.size != 3:
                raise ValueError("An 1x3 npdarray must be specified to move a single arm!")
            return update_component(component_name, jnt_values)
        elif component_name == 'both_arm':
            if (jnt_values.size != 7):
                raise ValueError("A 1x6 npdarrays must be specified to move both arm!")
            status_lft = update_component('lftfinger', jnt_values[0:3])
            status_rgt = update_component('rgtfinger', jnt_values[3:7])
            return "succ" if status_lft == "succ" and status_rgt == "succ" else "out_of_rng"
        elif component_name == 'all':
            raise NotImplementedError
        else:
            raise ValueError("The given component name is not available!")

    def gen_stickmodel(self,
                       tcp_jnt_id=None,
                       tcp_loc_pos=None,
                       tcp_loc_rotmat=None,
                       toggle_tcpcs=False,
                       toggle_jntscs=False,
                       toggle_connjnt=False,
                       name='ur3e_dual_stickmodel'):
        stickmodel = mc.ModelCollection(name=name)
        self.lftfinger.gen_stickmodel(tcp_jnt_id=tcp_jnt_id,
                                    tcp_loc_pos=tcp_loc_pos,
                                    tcp_loc_rotmat=tcp_loc_rotmat,
                                    toggle_tcpcs=toggle_tcpcs,
                                    toggle_jntscs=toggle_jntscs,
                                    toggle_connjnt=toggle_connjnt).attach_to(stickmodel)
        self.rgtfinger.gen_stickmodel(tcp_jnt_id=tcp_jnt_id,
                                    tcp_loc_pos=tcp_loc_pos,
                                    tcp_loc_rotmat=tcp_loc_rotmat,
                                    toggle_tcpcs=toggle_tcpcs,
                                    toggle_jntscs=toggle_jntscs,
                                    toggle_connjnt=toggle_connjnt).attach_to(stickmodel)
        return stickmodel

    def gen_meshmodel(self,
                      tcp_jnt_id=None,
                      tcp_loc_pos=None,
                      tcp_loc_rotmat=None,
                      toggle_tcpcs=False,
                      toggle_jntscs=False,
                      rgba_base=None,
                      rgba_lftfinger=None,
                      rgba_rgtfinger=None,
                      name='ur3e_dual_meshmodel'):
        mm_collection = mc.ModelCollection(name=name)
        self.base.gen_meshmodel(tcp_loc_pos=None,
                                    tcp_loc_rotmat=None,
                                    toggle_tcpcs=False,
                                    toggle_jntscs=toggle_jntscs,
                                    rgba=rgba_base).attach_to(mm_collection)
        self.lft.gen_meshmodel(tcp_loc_pos=None,
                                    tcp_loc_rotmat=None,
                                    toggle_tcpcs=False,
                                    toggle_jntscs=toggle_jntscs,
                                    rgba=rgba_base).attach_to(mm_collection)
        self.rgt.gen_meshmodel(tcp_loc_pos=None,
                                    tcp_loc_rotmat=None,
                                    toggle_tcpcs=False,
                                    toggle_jntscs=toggle_jntscs,
                                    rgba=rgba_base).attach_to(mm_collection)
        self.lftfinger.gen_meshmodel(tcp_jnt_id=tcp_jnt_id,
                                   tcp_loc_pos=tcp_loc_pos,
                                   tcp_loc_rotmat=tcp_loc_rotmat,
                                   toggle_tcpcs=toggle_tcpcs,
                                   toggle_jntscs=toggle_jntscs,
                                   rgba=rgba_lftfinger).attach_to(mm_collection)
        self.rgtfinger.gen_meshmodel(tcp_jnt_id=tcp_jnt_id,
                                   tcp_loc_pos=tcp_loc_pos,
                                   tcp_loc_rotmat=tcp_loc_rotmat,
                                   toggle_tcpcs=toggle_tcpcs,
                                   toggle_jntscs=toggle_jntscs,
                                   rgba=rgba_rgtfinger).attach_to(mm_collection)

        for obj_info in self.lft_oih_infos:
            objcm = obj_info['collision_model']
            objcm.set_pos(obj_info['gl_pos'])
            objcm.set_rotmat(obj_info['gl_rotmat'])
            objcm.copy().attach_to(mm_collection)
        for obj_info in self.rgt_oih_infos:
            objcm = obj_info['collision_model']
            objcm.set_pos(obj_info['gl_pos'])
            objcm.set_rotmat(obj_info['gl_rotmat'])
            objcm.copy().attach_to(mm_collection)
        return mm_collection

    def jaw_width(self,width,height):
        start_conf = np.array([0, 0, 0])
        goal_jnt_values = start_conf + np.array([math.pi * 1 / 8, math.pi * 1 / 8, math.pi * 1 / 8])
        pos = np.array([0, width/2, height])
        rot = np.array([[1, 0,  0],
                        [0, 0, -1],
                        [0, 1,  0]])
        goal_jnt_values = self.ik(tgt_pos=pos, tgt_rotmat=rot, seed_jnt_values=goal_jnt_values,
                                     component_name="lftfinger")
        goal_jnt_values = np.array([goal_jnt_values[0],goal_jnt_values[1],goal_jnt_values[2],goal_jnt_values[0],goal_jnt_values[1],goal_jnt_values[2]])
        self.fk("both_arm", jnt_values=goal_jnt_values)
        self.gen_meshmodel(rgba_rgtfinger=(1, 0, 0, 1),rgba_lftfinger=(1, 0, 0, 1),toggle_tcpcs=True, toggle_jntscs=True).attach_to(base)
        self.jaw_center_pos = [0,0,height]
        print(goal_jnt_values)

    def ssss(self):
        self.fk("both_arm", jnt_values=np.array([0.13757838,0.73808158,0.48650878,0.13757838,0.73808158,0.48650878,0]))
        self.gen_meshmodel( toggle_tcpcs=False,
                           toggle_jntscs=False).attach_to(base)

if __name__ == '__main__':
    import visualization.panda.world as wd
    import modeling.geometric_model as gm

    base = wd.World(cam_pos=[5, 0, 3], lookat_pos=[0, 0, 1])
    gm.gen_frame().attach_to(base)
    hand_gripper = Handgripper()
    hand_gripper.ssss()
    print(hand_gripper.jaw_center_pos)
    a= hand_gripper.base.lnks[0]['gl_rotmat']
    print(a)
    base.run()
