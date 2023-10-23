import os
import copy
import time

import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
import spatialmath.base as smb
import spatialgeometry as geometry

from swift import Swift
# from robot.m_DHRobot3D import M_DHRobot3D
from m_DHRobot3D import M_DHRobot3D
import ir_support

class Sawyer(M_DHRobot3D):

    """
    Sawyer robot.

        Parameters:
        --------------------------------------------
        gripper_ready: Indicator for mounting gripper.


    """
    _NEUTRAL = [0.00, -1.18, 0.00, -2.18, 0.00, -0.97, 3.3161]
    _script_directory = os.path.dirname(os.path.abspath(__file__))

    # -----------------------------------------------------------------------------------#
    #

    def __init__(
        self,
        env: Swift,
        base=sm.SE3(0, 0, 0),
        gripper_ready=False,
        gui=None,
    ):
        # DH links
        links = self._create_DH()
        self._gripper_ready = gripper_ready
        self._env = env

        # Names of the robot link files in the directory
        link3D_names = dict(
            link0="base",
            link1="l0",
            link2="l1",
            link3="l2",
            link4="l3",
            link5="l4",
            link6="l5",
            link7="l6",
        )

        # A joint config and the 3D object transforms to match that config
        qtest = [0, -np.pi/2, 0, 0, 0, 0, np.pi]
        qtest_transforms = [
            smb.transl(0, 0, 0),
            smb.transl(0, 0, 0.079106),
            smb.transl(0.081601, 0.049653, 0.315798) @ smb.trotx(-np.pi/2),
            smb.transl(0.081787, 0.191725, 0.455567),
            smb.transl(0.081217, 0.149346, 0.71623) @ smb.trotx(-np.pi/2),
            smb.transl(0.081662, 0.023241, 0.841128),
            smb.transl(0.081395, 0.054012, 1.11607) @ smb.trotx(-np.pi/2),
            smb.transl(0.081361, 0.160295, 1.22569),
        ]

        current_path = os.path.abspath(os.path.dirname(__file__))
        current_path = os.path.join(current_path, "Sawyer_model")
        super().__init__(
            links,
            link3D_names,
            name="Sawyer",
            link3d_dir=current_path,
            qtest=qtest,
            qtest_transforms=qtest_transforms,
        )

        self.base = base * self.base
        self.q = qtest
        self._head = self._add_model(
            "Sawyer_model/head.dae", smb.transl(0, 0, 0.3811))
        self.set_neutral_js(self._NEUTRAL)

        # Add gripper
        self.gripper = Gripper_Sawyer(env, sawyer_ee= self.fkine(self.q))
        self.update_sim()


    def _create_DH(self):
        """
        Create robot's standard DH model
        """

        # deg = np.pi / 180
        mm = 1e-3

        # kinematic parameters
        a = np.r_[81, 0, 0, 0, 0, 0, 0] * mm
        d = np.r_[317, 192.5, 400, 168.5, 400, 136.3, 133.75] * mm
        alpha = [-np.pi / 2, -np.pi / 2, -np.pi /
                 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, 0]
        qlim = np.deg2rad([[-175, 175],
                           [-175, 175],
                           [-175, 175],
                           [-175, 175],
                           [-170.5, 170.5],
                           [-170.5, 170.5],
                           [-270, 270]])

        # offset to have the dh from toolbox match with the actual pose
        offset = [0, -np.pi/2, 0, np.pi, 0, np.pi, 0]

        links = []

        for j in range(7):
            link = rtb.RevoluteDH(
                d=d[j], a=a[j], alpha=alpha[j], offset=offset[j], qlim=qlim[j])
            links.append(link)

        return links

    def _add_model(self, file_path, placement, color=None):
        """
        Add model to simulation environment
        """
        model_full_path = os.path.join(self._script_directory, file_path)
        model_placement = self.base.A @ placement
        if color is None:
            model = geometry.Mesh(
                model_full_path, pose=model_placement,collision=True)
        else:
            model = geometry.Mesh(
                model_full_path, pose=model_placement,collision=True, color=color)

        self._env.add(model, collision_alpha=1)
        return model

    def update_sim(self):
        """
        Reconfigure robot to home position set by user

        """
        self.add_to_env(self._env)
        if self._gripper_ready:
            self.gripper.base = self.get_jointstates()
            self.gripper.add_to_env(self._env)


    def get_workspace(self):
        """
        Get workspace of the robot
        """
        step = np.pi / 4
        pointcloud_size = 5 * int(
            np.prod(
                np.floor((self.qlim[1, 1:6] - self.qlim[0, 1:6]) / step + 1))
        )
        print(pointcloud_size)

        pointcloud = np.zeros((pointcloud_size, 3))
        counter = 0
        start_time = time.time()
        print("Start getting point cloud...")

        for q0 in np.arange(self.qlim[0, 0], self.qlim[1, 0] + 0.2, 0.2):
            for q1 in np.arange(self.qlim[0, 1], self.qlim[1, 1] + step, step):
                for q2 in np.arange(self.qlim[0, 2], self.qlim[1, 2] + step, step):
                    for q3 in np.arange(self.qlim[0, 3], self.qlim[1, 3] + step, step):
                        for q4 in np.arange(self.qlim[0, 4], self.qlim[1, 4] + step, step):
                            for q5 in np.arange(self.qlim[0, 5], self.qlim[1, 5] + step, step):

                                q = [q0, q1, q2, q3, q4, q5, 0]

                                ep = self.fkine(q).A
                                if counter == pointcloud_size:
                                    break
                                pointcloud[counter, :] = ep[0:3, 3]
                                counter += 1
                                if np.mod(counter / pointcloud_size * 100, 1) == 0:
                                    end_time = time.time()
                                    execution_time = end_time - start_time
                                    print(
                                        f"After {execution_time} seconds, complete",
                                        counter / pointcloud_size * 100,
                                        "% of pose",
                                    )

        # Eliminate points that lie below the table
        pointcloud = pointcloud[pointcloud[:, 2] > 0.1+self.base.A[2, 3], :]
        pc_x = pointcloud[:, 0]
        pc_y = pointcloud[:, 1]
        print('max x: ', np.max(pc_x), 'min x: ', np.min(pc_x))
        print('max y: ', np.max(pc_y), 'min y: ', np.min(pc_y))
        print("Point cloud complete with %d points captured!", len(pointcloud))
        return pointcloud
    
    def send_joint_command(self, q):
        """
        Send joint command to robot. Current mode available is joint position mode
        """
        self.q = q
        if self._gripper_ready:
            self.gripper.base = self.fkine(self.get_jointstates())
        self._env.step(0)


    def rotate_head(self, angle):
        """
        animate head motion

        """
        head_from_base = np.linalg.inv(self.base.A) @ self._head.T
        head_ori = smb.tr2rpy(copy.deepcopy(
            self._head.T[0:3, 0:3]), order="xyz")
        step = (angle - head_ori[2]) / (np.pi/48)
        step = np.pi/48 * step / abs(step)
        for i in np.arange(head_ori[2], angle, step):
            self._head.T = smb.trotz(i) @ head_from_base
            self._env.step(0.02)
            
    def move_gripper(self, sawyer_ee):
        self.gripper.move_gripper(sawyer_ee= sawyer_ee)
        self.gripper.Gripper_Left._update_3dmodel()
        self.gripper.Gripper_Right._update_3dmodel()

    # -----------------------------------------------------------------------------------#
    # ENV TESTING
    def test(self):
        """
        Test the class by adding 3d objects into a new Swift window and do a simple movement
        """

        self.q = self._qtest
        self.update_sim()

        q_goal = [0.00, -1.18, 0.00, -2.18, 0.00, 0.57-np.pi/2, 3.3161]
        qtraj = rtb.jtraj(self.q, q_goal, 10).q

        # fig = self.plot(self.q, limits= [-1,1,-1,1,-1,1])
        # fig._add_teach_panel(self, self.q)
        for q in qtraj:
            self.q = q  # Robot jointstate
            self._env.step(0.02)
        #     fig.step(0.01)
        # fig.hold()

        # self._env.hold()


# ---------------------------------------------------------------------------------------#
class Gripper_Sawyer_left(M_DHRobot3D):
    _script_directory = os.path.dirname(os.path.abspath(__file__))
    
    def __init__(
        self,
        env: Swift,
        sawyer_ee
    ):
        links = self._create_DH()
        self._env = env

        
        link3D_names = dict(
            link0 = "sawyer_gripper_base",
            link1 = "sawyer_gripper_left"
        )
        
        qtest = [0]
        qtest_transforms = [
            smb.transl(0,0,0) @ smb.troty(-np.pi/2),
            smb.transl(-0.0715,0.0042,-0.0029)
        ]
        
        current_path = os.path.abspath(os.path.dirname(__file__))
        current_path = os.path.join(current_path, "Sawyer_model")
        super().__init__(
            links,
            link3D_names,
            name= "Gripper_Sawyer_left",
            link3d_dir= current_path,
            qtest= qtest,
            qtest_transforms= qtest_transforms,
        )

        self.q = qtest
        self._base_origin_left = self.base * sm.SE3.Ry(90,'deg') * sm.SE3(0.007,0,0)
        self.set_position(sawyer_ee= sawyer_ee)
        self.update_sim()
        
    def update_sim(self):
        self.add_to_env(self._env)

    def _create_DH(self):
        links = []
        
        link = rtb.PrismaticDH(alpha= 0, offset= 0, qlim= [-24.19, 24.19])
        links.append(link)
        return links
    
    def set_position(self, sawyer_ee):
        self.base = sawyer_ee * self._base_origin_left

    def _add_model(self, file_path, placement, color=None):
        """
        Add model to simulation environment
        """
        model_full_path = os.path.join(self._script_directory, file_path)
        model_placement = self.base.A @ placement
        if color is None:
            model = geometry.Mesh(
                model_full_path, pose=model_placement,collision=True)
        else:
            model = geometry.Mesh(
                model_full_path, pose=model_placement,collision=True, color=color)

        self._env.add(model, collision_alpha=1)
        return model

    # -----------------------------------------------------------------------------------#
    # ENV TESTING
    def test(self):
        """
        Test the class by adding 3d objects into a new Swift window and do a simple movement
        """

        self.q = self._qtest

        while True:
            q_goal = [-0.024]
            qtraj = rtb.jtraj(self.q, q_goal, 10).q
            for q in qtraj:
                self.q = q  # Robot jointstate
                self._env.step(0.02)
                # fig.step(0.01)
                
            q_goal = [0.024]
            qtraj = rtb.jtraj(self.q, q_goal, 10).q
            for q in qtraj:
                self.q = q  # Robot jointstate
                self._env.step(0.02)
                # fig.step(0.01)
                
            time.sleep(0.2)

    
class Gripper_Sawyer_right(M_DHRobot3D):
    _script_directory = os.path.dirname(os.path.abspath(__file__))

    def __init__(
        self,
        env: Swift,
        sawyer_ee
    ):
        links = self._create_DH()
        self._env = env
        
        link3D_names = dict(
            link0 = "sawyer_gripper_base",
            link1 = "sawyer_gripper_right"
        )
        
        qtest = [0]
        qtest_transforms = [
            smb.transl(0,0,0) @ smb.troty(-np.pi/2),
            smb.transl(-0.0715,0.0038,-0.0029)
        ]
        
        current_path = os.path.abspath(os.path.dirname(__file__))
        current_path = os.path.join(current_path, "Sawyer_model")
        super().__init__(
            links,
            link3D_names,
            name= "Gripper_Sawyer_left",
            link3d_dir= current_path,
            qtest= qtest,
            qtest_transforms= qtest_transforms,
        )

        self.q = qtest
        
        self._base_origin_right = self.base * sm.SE3.Ry(90,'deg') * sm.SE3(0.007,0,0)
        self.set_position(sawyer_ee= sawyer_ee)
        self.update_sim()

    def update_sim(self):
        self.add_to_env(self._env)
        
    def _create_DH(self):
        links = []
        link = rtb.PrismaticDH(alpha= 0, offset= 0, qlim= [-24.19, 24.19])
        links.append(link)
        return links
    
    def set_position(self, sawyer_ee):
        self.base = sawyer_ee * self._base_origin_right
        
    def _add_model(self, file_path, placement, color=None):
        """
        Add model to simulation environment
        """
        model_full_path = os.path.join(self._script_directory, file_path)
        model_placement = self.base.A @ placement
        if color is None:
            model = geometry.Mesh(
                model_full_path, pose=model_placement,collision=True)
        else:
            model = geometry.Mesh(
                model_full_path, pose=model_placement,collision=True, color=color)

        self._env.add(model, collision_alpha=1)
        return model
    
    # -----------------------------------------------------------------------------------#
    # ENV TESTING
    def test(self):
        """
        Test the class by adding 3d objects into a new Swift window and do a simple movement
        """

        self.q = self._qtest

        while True:
            q_goal = [-0.024]
            qtraj = rtb.jtraj(self.q, q_goal, 10).q
            for q in qtraj:
                self.q = q  # Robot jointstate
                self._env.step(0.02)
                # fig.step(0.01)
                
            q_goal = [0.024]
            qtraj = rtb.jtraj(self.q, q_goal, 10).q
            for q in qtraj:
                self.q = q  # Robot jointstate
                self._env.step(0.02)
                # fig.step(0.01)
                
            time.sleep(0.2)
            

class Gripper_Sawyer():
    def __init__(
        self,
        env: Swift,
        sawyer_ee
    ):
        self._env = env
        self.Gripper_Right = Gripper_Sawyer_right(self._env, sawyer_ee)
        self.Gripper_Left = Gripper_Sawyer_left(self._env, sawyer_ee)
        
        self.Gripper_Right.q = [0]
        self.Gripper_Left.q = [0]

    def update_sim(self):
        self.Gripper_Right.add_to_env(self._env)
        self.Gripper_Left.add_to_env(self._env)
            
    def close_gripper(self):
        q_goal = np.array([0.03])
        steps = 20
        qtraj_left = rtb.jtraj(self.Gripper_Left.q, -1*q_goal,steps).q
        qtraj_right = rtb.jtraj(self.Gripper_Right.q, q_goal,steps).q
        
        for i in range (steps):
            self.Gripper_Left.q = qtraj_left[i]
            self.Gripper_Right.q = qtraj_right[i]
            self._env.step(0.02)
    
    def open_gripper(self):
        q_goal = np.array([0])
        steps = 20
        qtraj_left = rtb.jtraj(self.Gripper_Left.q, q_goal,steps).q
        qtraj_right = rtb.jtraj(self.Gripper_Right.q, q_goal,steps).q
        
        for i in range (steps):
            self.Gripper_Left.q = qtraj_left[i]
            self.Gripper_Right.q = qtraj_right[i]
            self._env.step(0.02) 
        
    def move_gripper(self, sawyer_ee):
        self.Gripper_Left.set_position(sawyer_ee= sawyer_ee)
        self.Gripper_Right.set_position(sawyer_ee= sawyer_ee)
        
        
if __name__ == "__main__":

    # generate environment
    env = Swift()
    env.launch(realtime=True)

    # generate robot
    r = Sawyer(env)
    # r.test()

    
    time.sleep(2)
    q_goal = [0.00, -1.18, 0.00, -2.18, 0.00, 0.57-np.pi/2, 3.3161]
    qtraj = rtb.jtraj(r.q, q_goal, 200).q
    for q in qtraj:
        r.q = q
        r.move_gripper(r.fkine(r.q))
        env.step(0.02)
    
    while True:
        time.sleep(0.2)
        r.gripper.close_gripper()
        time.sleep(0.2)
        r.gripper.open_gripper()
        
    env.hold()

