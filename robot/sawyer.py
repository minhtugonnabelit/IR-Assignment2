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
    Sawyer robot.

        Parameters:
        --------------------------------------------
        gripper_ready: Indicator for mounting gripper.


    """
    _neutral = [0.00, -1.18, 0.00, -2.18, 0.00, 0.57, 3.3161]
    _script_directory = os.path.dirname(os.path.abspath(__file__))

    # -----------------------------------------------------------------------------------#
    #

    def __init__(
        self,
        env: Swift,
        base=sm.SE3(0, 0, 0),
        gripper_ready=True,
        gui=None,
    ):
        # DH links
        links = self._create_DH()
        self._gripper_ready = gripper_ready
        self._env = env
        self._UIjs = np.zeros(7)

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
        self._head = self._add_model("Sawyer_model/head.DAE", smb.transl(0, 0, 0.3811))
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
        alpha = [-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, 0]
        qlim = [[-2*np.pi, 2*np.pi] for _ in range(7)]

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
                model_full_path, pose=model_placement,)
        else:
            model = geometry.Mesh(
                model_full_path, pose=model_placement, color=color)
            
        self._env.add(model, collision_alpha=1)
        return model

    def update_sim(self):
        """
        Reconfigure robot to home position set by user

        """
        self._ellipsoids = self.get_ellipsoid()
        self.add_to_env(self._env)

    def set_base(self, base):
        """
        Set base for system based on user input
        """
        self.base = base * self.base

    def get_ee_pose(self):
        """
        Get end-effector pose of the robot
        """
        return self.fkine(self.q)

    def get_jointstates(self):
        """
        Get robot joint states
        """
        return copy.deepcopy(self.q)

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

    # COLISION and SAFETY FUNCTION
    # -----------------------------------------------------------------------------------#
    
    def stop(self):
        """
        Immediately Stop the robot motion
        """
        
        pass

    def get_ellipsoid(self):
        """
        Get ellipsoid of the robot
        """

        ellipsoids = []
        for i in range(len(self.links)):

            minor_axis = 0.03 if self.a[i] == 0 else copy.deepcopy(self.a[i]) / 2 + 0.02
            major_axis = copy.deepcopy(self.d[i]) / 2

            ellipsoid = np.asarray(
                [minor_axis, major_axis, minor_axis])
            
            ellipsoids.append(ellipsoid)

        ellipsoids[6][2] = ellipsoids[6][1]
        ellipsoids[6][1] = ellipsoids[6][0]
        return ellipsoids

    def get_link_poses(self, q=None):
        """
        :param q robot joint angles
        :param robot -  seriallink robot model
        :param transforms - list of transforms
        """
        if q is None:
            return self.fkine_all().A
        return self.fkine_all(q).A
    
    def is_grounded(self, q=None):
        """
        Check if robot is grounded
        """
        pass

    # testing
    def is_collided_with_object(self, q=None):
        """
        Check if robot is collided with object
        """

        pass
    
    # testing
    def self_collided_LP(self,):
        """
        Check self-collision of the robot using line-plance intersection
        """
        is_collided = False
        tr = self.get_link_poses()

        for i in range(len(tr)):

            if i == 0:
                continue

            center = (tr[i][0:3, 3] + tr[i-1][0:3, 3])/2
            radi = copy.deepcopy(self._ellipsoids[i-1])
            elip = np.round((tr[i][0:3, 0:3] @ np.diag(np.power(radi, -2)) @ np.transpose(tr[i][0:3, 0:3])), 3)
            ob = smb.plot_ellipsoid(elip, center, resolution=10, color='r', alpha=0.5)

            for j in range(len(tr)):
                if j == i or j == i-1:
                    continue
                center = (tr[j][0:3, 3] + tr[j-1][0:3, 3])/2
                radi = copy.deepcopy(self._ellipsoids[j-1])
                elip = np.round((tr[j][0:3, 0:3] @ np.diag(np.power(radi, -2)) @ np.transpose(tr[j][0:3, 0:3])), 3)
                ob = smb.plot_ellipsoid(elip, center, resolution=10, color='r', alpha=0.5)

                if ob.intersect(ob):
                    is_collided = True
                    break


        return is_collided
    
    # testing
    def self_collisions_Cylinder(self, q=None):
        """
        Check self-collision of the robot using cylinder intersection
        """
        is_collided = False
        tr = self.get_link_poses(q)

        for i in range(len(tr)):

            if i == 0:
                continue

            center = (tr[i][0:3, 3] + tr[i-1][0:3, 3])/2
            radi = copy.deepcopy(self._ellipsoids[i-1])
            ob = geometry.Cylinder(center, radi[0], radi[1])
            for j in range(len(tr)):
                if j == i or j == i-1:
                    continue
                center = (tr[j][0:3, 3] + tr[j-1][0:3, 3])/2
                radi = copy.deepcopy(self._ellipsoids[j-1])
                ob2 = geometry.Cylinder(center, radi[0], radi[1])
                if ob.intersect(ob2):
                    is_collided = True
                    break

        return is_collided


    # MOTION FUNCTION
    # -----------------------------------------------------------------------------------#
    # def go_to_CartesianPose(self, pose : SE3, time=1, tolerance=0.0001):
        
    #     """
    #     ### Move robot to desired Cartesian pose
    #     Function to move robot end-effector to desired Cartesian pose. 
    #     This function is performing tehcnique called RMRC (Resolved Motion Rate Control) to move robot to desired pose.
    #     - @param pose: desired Cartesian pose           
    #     - @param time: time to complete the motion
    #     - @param tolerance: tolerance to consider the robot has reached the desired pose

    #     """

    #     step = 100
    #     time_step = time/step
    #     current_ee_pose = self.get_ee_pose()

    #     path = rtb.ctraj(current_ee_pose, pose, t=step)

    #     # for i in range(len(path)-1):

    #     #     prev_ee_pos = path[i].A[0:3, 3]
    #     #     desired_ee_pos = path[i+1].A[0:3, 3]

    #     #     # get linear velocity between interpolated point and current pose of ee
    #     #     lin_vel = (desired_ee_pos - prev_ee_pos) / time_step

    #     #     # get angular velocity between interpolated ...
    #     #     s_omega = (path[i+1].A[0:3, 0:3] @ np.transpose(
    #     #         self.get_ee_pose().A[0:3, 0:3]) - np.eye(3)) / time_step
    #     #     ang_vel = [s_omega[2, 1], s_omega[0, 2], s_omega[1, 0]]

    #     #     # combine vel
    #     #     ee_vel = np.hstack((lin_vel, ang_vel))

    #     #     # get joint velocities
    #     #     joint_vel = np.linalg.pinv(
    #     #         self.jacob0(self.q)) @ np.transpose(ee_vel)

    #     #     current_js = self.get_jointstates()
    #     #     q = current_js + joint_vel * time_step
    #     #     self.q = q

    #     #     self._env.step(time_step)

    #     # get ee carterian pose difference wih desired pose
    #     def is_arrived():
    #         ee_pose = self.get_ee_pose()
    #         poss_diff = np.diff(ee_pose.A - pose.A)
    #         if np.all(np.abs(poss_diff) < tolerance):
    #             print('done')
    #             return True

    #         return False

    #     index = 0
    #     while not is_arrived():

    #         prev_ee_pos = path[index].A[0:3, 3]
    #         desired_ee_pos = path[index+1].A[0:3, 3]

    #         # get linear velocity between interpolated point and current pose of ee
    #         lin_vel = (desired_ee_pos - prev_ee_pos) / time_step

    #         # get angular velocity between interpolated ...
    #         s_omega = (path[index+1].A[0:3, 0:3] @ np.transpose(
    #             self.get_ee_pose().A[0:3, 0:3]) - np.eye(3)) / time_step
    #         ang_vel = [s_omega[2, 1], s_omega[0, 2], s_omega[1, 0]]

    #         # combine velocities
    #         ee_vel = np.hstack((lin_vel, ang_vel))

    #         # get joint velocities
    #         joint_vel = np.linalg.pinv(
    #             self.jacob0(self.q)) @ np.transpose(ee_vel)

    #         #@todo:
    #         # -- adding sigulariry check and joint limit check
    #         # -- consider adding collision check   
            
    #         # update joint states as a command to robot
    #         current_js = self.get_jointstates()
    #         q = current_js + joint_vel * time_step
    #         self.q = q

    #         index += 1
    #         if index == len(path)-1 and not is_arrived():
    #             print('Pose is unreachable!')
    #             break

    #         self._env.step(time_step)


    # # -----------------------------------------------------------------------------------#
    # def ee_plus_z(self):
    #     """
    #     move ee in z direction locally by 0.01m
    #     """
    #     pose = self.get_ee_pose() @ sm.SE3.Tz(0.05)
    #     self.go_to_CartesianPose(pose, time=0.02)

    # def ee_minus_z(self):
    #     """
    #     move ee in z direction locally by -0.01m
    #     """  
    #     pose = self.get_ee_pose() @ sm.SE3.Tz(-0.05)
    #     self.go_to_CartesianPose(pose, time=0.02)

    # def ee_plus_x(self):
    #     """
    #     move ee in x direction locally by 0.01m
    #     """
    #     pose = self.get_ee_pose() @ sm.SE3.Tx(0.05)
    #     self.go_to_CartesianPose(pose, time=0.02)

    # def ee_minus_x(self):

    #     """
    #     move ee in x direction locally by -0.01m
    #     """
    #     pose = self.get_ee_pose() @ sm.SE3.Tx(-0.05)
    #     self.go_to_CartesianPose(pose, time=0.02)

    # def ee_plus_y(self):
    #     """
    #     move ee in y direction locally by 0.01m
    #     """
    #     pose = self.get_ee_pose() @ sm.SE3.Ty(0.05)
    #     self.go_to_CartesianPose(pose, time=0.02)

    # def ee_minus_y(self):

    #     """
    #     move ee in y direction locally by -0.01m
    #     """
    #     pose = self.get_ee_pose() @ sm.SE3.Ty(-0.05)
    #     self.go_to_CartesianPose(pose, time=0.02)

    # ## orientation
    # def ee_plus_z_ori(self):
    #     """
    #     move ee in z direction locally by 0.1 radians
    #     """

    #     for i in range(50):
    #         self.q[6] = self.q[6] + 0.2/50
    #         self.q = self.q
    #         self._env.step(0.01/50)

    # def ee_minus_z_ori(self):
    #     """
    #     move ee in z direction locally by -0.1 radians
    #     """

    #     for i in range(50):
    #         self.q[6] = self.q[6] - 0.2/50
    #         self.q = self.q
    #         self._env.step(0.01/50)

    # def ee_plus_x_ori(self):
    #     """
    #     move ee in x direction locally by 0.1 radians
    #     """
    #     pose = self.get_ee_pose() @ sm.SE3.Rx(0.2)
    #     self.go_to_CartesianPose(pose, time=0.01)
    
    # def ee_minus_x_ori(self):
    #     """
    #     move ee in x direction locally by -0.1 radians
    #     """
    #     pose = self.get_ee_pose() @ sm.SE3.Rx(-0.2)
    #     self.go_to_CartesianPose(pose, time=0.01)
    
    # def ee_plus_y_ori(self):
    #     """
    #     move ee in y direction locally by 0.1 radians
    #     """
    #     pose = self.get_ee_pose() @ sm.SE3.Ry(0.2)
    #     self.go_to_CartesianPose(pose, time=0.01)
    
    # def ee_minus_y_ori(self):
    #     """
    #     move ee in y direction locally by -0.1 radians
    #     """
    #     pose = self.get_ee_pose() @ sm.SE3.Ry(-0.2)
    #     self.go_to_CartesianPose(pose, time=0.01)
    
    # -----------------------------------------------------------------------------------#
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

    def home(self):
        """
        Reconfigure robot to home position set by user

        """
        home_traj = rtb.jtraj(self.q, self._neutral, 100)
        for q in home_traj.q:
            self.q = q
            self._env.step(0.02)

    def move_to_joint_position(self, q_desired, step=100):
        """
        Reconfigure robot to home position set by user

        """
        traj = rtb.jtraj(self.q, q_desired, step)
        for q in traj.q:
            self.q = q
            self._env.step(0.02)

    def move(self):
        """
        Execute a joint space trajectory

        """
        path = rtb.jtraj(self.q, self._UIjs, 100)
        for q in path.q:
            self.q = q
            self._env.step(0.02)

    def set_joint_value(self, j, value):
        """
        Set joint value

        """
        self._UIjs[j] = np.deg2rad(float(value))
        return self._UIjs


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

    def plot_elipsoids(self):
        """
        Test ellipsoid function
        """
        tr = self.get_link_poses()
        for i in range(len(tr)):

            if i == 0:
                continue

            center = (tr[i][0:3, 3] + tr[i-1][0:3, 3])/2
            elip = np.round((tr[i][0:3, 0:3] @ np.diag(np.power(self._ellipsoids[i-1], -2)) @ np.transpose(tr[i][0:3, 0:3])), 3)
            ob = smb.plot_ellipsoid(elip, center, resolution=10, color='r', alpha=0.5)


# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":

    # generate environment
    env = Swift()
    env.launch(realtime=True)

    # generate robot
    r = Sawyer(env)
    r.test()
