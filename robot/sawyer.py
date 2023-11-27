import os
import copy
import time

import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
import spatialmath.base as smb
import spatialgeometry as geometry

from swift import Swift
from robot.m_DHRobot3D import M_DHRobot3D


class Sawyer(M_DHRobot3D):

    """
    
        Sawyer
        --------------------------------------------
        env: Environment to add robot to.
        base: Base pose of robot.
        gripper_ready: Indicator for mounting gripper.

    """
    # _NEUTRAL = [0.00, -0.82, 0.00, 2.02, 0.00, -1.22,  1.57]  
    #            [0.00, -0.9, 0.00, -1.9, 0.00, -0.97, 3.14] 
    #            [0.00, -1.18, 0.00, -2.18, 0.00, -0.97, 3.14]

    _NEUTRAL= np.deg2rad(np.array([-51.26, -40.96, -7.47, 83.18, -61.48, -69.74, 134.15]))
    _script_directory = os.path.dirname(os.path.abspath(__file__))

    # -----------------------------------------------------------------------------------#
    #

    def __init__(
        self,
        env: Swift,
        base=sm.SE3(0, 0, 0),
        gripper_ready=False,
    ):

        # Initialize environment to update robot body
        self._env = env


        # Initialize gripper parameters
        self._gripper_ready = gripper_ready
        self._offset_gripper = 0.16 if self._gripper_ready else 0


        # DH links
        links = self._create_DH()


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
        self.gripper = SawyerGripper(base=self.fkine(self.q))
        self.ax = geometry.Axes(0.2, pose=self.fkine(self.q))
        self.update_sim()

    def _create_DH(self):
        """
        Create robot's standard DH model
        """

        mm = 1e-3

        # kinematic parameters
        a = np.r_[81, 0, 0, 0, 0, 0, 0] * mm
        d = np.r_[317, 192.5, 400, -168.5, 400, 136.3,
                  133.75 + self._offset_gripper/mm] * mm

        alpha = [-np.pi / 2,
                 np.pi / 2,
                 -np.pi / 2,
                 np.pi / 2,
                 -np.pi / 2,
                 np.pi / 2,
                 0]
        qlim = np.deg2rad([[-175, 175],
                           [-219, 131],
                           [-175, 175],
                           [-175, 175],
                           [-170.5, 170.5],
                           [-170.5, 170.5],
                           [-270, 270]])

        # offset to have the dh from toolbox match with the actual pose
        offset = [0, np.pi/2, 0, 0, 0, 0, -np.pi/2]


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
                model_full_path, pose=model_placement, collision=True)
        else:
            model = geometry.Mesh(
                model_full_path, pose=model_placement, collision=True, color=color)

        self._env.add(model, collision_alpha=1)
        return model
    

    def update_sim(self):
        """
        Reconfigure robot to home position set by user

        """
        self.add_to_env(self._env)
        self._env.add(self.ax)
        if self._gripper_ready:
            self.gripper.base = self.fkine(self.get_jointstates())
            self.gripper.add_to_env(self._env)


    def send_joint_command(self, q):
        """
        Send joint command to robot. Current mode available is joint position mode
        """
        self.q = q
        self.ax.T = self.fkine(self.q)
        if self._gripper_ready:
            self.gripper.base = self.fkine(self.get_jointstates())


    def open_gripper(self):
        """
        Function to open gripper
        """
        self.gripper.open()


    def close_gripper(self):
        """
        Function to close gripper
        """
        self.gripper.close()


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


class Finger(M_DHRobot3D):
    """
    Finger class is a subclass of DHRobot3D class
    """

    def __init__(self, base, links, link3D_names, name=None, qtest=None):

        qtest = [0]
        finger_transform = [
            smb.transl(0, 0, 0) @ smb.troty(-np.pi/2),
            smb.transl(-0.0715, 0.0042, -0.0029)
        ]

        current_path = os.path.abspath(os.path.dirname(__file__))
        gripper_path = os.path.join(current_path, "Sawyer_model")

        super().__init__(
            links,
            link3D_names,
            gripper_path,
            name,
            qtest,
            qtest_transforms=finger_transform,
        )
        self.base = base


class SawyerGripper:
    """
    Sawyer gripper class is a subclass of DHRobot3D class

    """

    _qtest = [0]

    @property
    def base(self):
        return self._base

    def __init__(self, base=sm.SE3(0, 0, 0)):

        links = self._create_DH()

        # This base is created just to initial the gripper model, then the main 'base' as a class property is used for asssiging the primary base
        base_init = sm.SE3(0, 0, 0)

        # set attribute here: when we use "base", base at _init_ has the value, then it will be assigned to function set_base(value) at attribute function
        self._base = base


        # Right finger properties

        right_link3D_names = dict(
            link0="sawyer_gripper_base",
            link1="sawyer_gripper_right"
        )

        self._right_finger = Finger(
            base_init,
            links,
            right_link3D_names,
            name="right_f",
            qtest=self._qtest,
        )

        self.base_tf_right = sm.SE3.Ry(90, 'deg') * sm.SE3(0.007 + 0.16, 0, 0)
        self._right_finger.base = (
            self._base.A @ self.base_tf_right.A
        )


        # Left finger properties

        left_link3D_names = dict(
            link0="sawyer_gripper_base",
            link1="sawyer_gripper_left"
        )

        self._left_finger = Finger(
            base_init,
            links,
            left_link3D_names,
            name="left_f",
            qtest=self._qtest
        )

        self.base_tf_left = sm.SE3.Ry(90, 'deg') * sm.SE3(0.007 + 0.16, 0, 0)
        self._left_finger.base = (
            self._base.A @ self.base_tf_left.A
        )

    # -----------------------------------------------------------------------------------#

    def close(self):
        """
        Function to close gripper model
        """
        q_goal = np.array([0.027])
        steps = 20
        qtraj_left = rtb.jtraj(self._left_finger.q, -1*q_goal, steps).q
        qtraj_right = rtb.jtraj(self._right_finger.q, q_goal, steps).q

        for i in range(steps):
            self._left_finger.q = qtraj_left[i]
            self._right_finger.q = qtraj_right[i]
            time.sleep(0.02)


    def open(self):
        """
        Function to open gripper model
        """
        q_goal = np.array([0])
        steps = 20
        qtraj_left = rtb.jtraj(self._left_finger.q, q_goal, steps).q
        qtraj_right = rtb.jtraj(self._right_finger.q, q_goal, steps).q

        for i in range(steps):
            self._left_finger.q = qtraj_left[i]
            self._right_finger.q = qtraj_right[i]
            time.sleep(0.02)

    # -----------------------------------------------------------------------------------#

    def _create_DH(self):
        """
        The gripper chosen to use for this mission is Onrobot gripper RG6

            Gripper model is constructed by one base with two fingers
            Base is considered as a statis object attached to robot end-effectorand transform along with ee pose .
            Two fingers are models as two 2-links plannar robots with a constraint for the ee of those plannar always align local z axis of the gripper base
        """
        links = []

        link = rtb.PrismaticDH(alpha=0, offset=0, qlim=[-24.19, 24.19])
        links.append(link)

        return links

    # -----------------------------------------------------------------------------------#

    def add_to_env(self, env):
        """
        Function to add 2 fingers model to environment
        """

        # add 2 fingers to the environment

        self._env = env
        self._right_finger.add_to_env(env)
        self._left_finger.add_to_env(env)

    # -----------------------------------------------------------------------------------#

    def set_base(self, newbase):
        """
        Function to update gripper base
        """

        # assign new base for the gripper to be attached with robot ee given

        self._right_finger.base = newbase.A @ self.base_tf_right.A
        self._left_finger.base = newbase.A @ self.base_tf_left.A

        # give empty q so that gripper model is updated on swift

        self._right_finger.q = self._right_finger.q
        self._left_finger.q = self._left_finger.q

    # -----------------------------------------------------------------------------------#

    def __setattr__(self, name, value):
        """
        Overload `=` operator so the object can update its 3D model whenever a new base is assigned
        """

        if name == "base" and hasattr(self, "base"):
            # Update the 3D model before setting the attribute
            self.set_base(value)
        else:
            super().__setattr__(
                name, value
            )  # Call the base class method to set the attribute


if __name__ == "__main__":

    # generate environment
    env = Swift()
    env.launch(realtime=True)

    # generate robot
    r = Sawyer(env, gripper_ready=True)

    time.sleep(0.5)
    # q_goal = [0.00, -1.18, 0.00, -2.18, 0.00, 0.57-np.pi/2, 3.3161]
    q_goal = r._NEUTRAL
    qtraj = rtb.jtraj(r.q, q_goal, 200).q
    for q in qtraj:
        r.send_joint_command(q)
        time.sleep(0.02)

    r.close_gripper()
    # time.sleep(1)
    r.open_gripper()
    env.hold()
