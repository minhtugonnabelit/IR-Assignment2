import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
import spatialmath.base as smb
from swift import Swift
import matplotlib.pyplot as plt

import time

class Controller():

    def __init__(self, robot : rtb.DHRobot, env : Swift, isSim=True) -> None:

        self._robot = robot
        self._env   = env
        self._isSim = isSim

        pass

    # MOTION FUNCTION
    # -----------------------------------------------------------------------------------#

    def go_to_CartesianPose(self, pose : sm.SE3, time=1, tolerance=0.0001):
        
        """
        ### Move robot to desired Cartesian pose
        Function to move robot end-effector to desired Cartesian pose. 
        This function is performing tehcnique called RMRC (Resolved Motion Rate Control) to move robot to desired pose.
        - @param pose: desired Cartesian pose           
        - @param time: time to complete the motion
        - @param tolerance: tolerance to consider the robot has reached the desired pose

        """

        step = 100
        time_step = time/step
        current_ee_pose = self._robot.get_ee_pose()

        path = rtb.ctraj(current_ee_pose, pose, t=step)

        # get ee carterian pose difference wih desired pose
        def is_arrived():
            ee_pose = self._robot.get_ee_pose()
            poss_diff = np.diff(ee_pose.A - pose.A)
            if np.all(np.abs(poss_diff) < tolerance):
                print('done')
                return True

            return False

        index = 0
        while not is_arrived():

            prev_ee_pos = path[index].A[0:3, 3]
            desired_ee_pos = path[index+1].A[0:3, 3]

            # get linear velocity between interpolated point and current pose of ee
            lin_vel = (desired_ee_pos - prev_ee_pos) / time_step

            # get angular velocity between interpolated ...
            s_omega = (path[index+1].A[0:3, 0:3] @ np.transpose(
                self._robot.fkine(self._robot.q).A[0:3, 0:3]) - np.eye(3)) / time_step
            ang_vel = [s_omega[2, 1], s_omega[0, 2], s_omega[1, 0]]

            # combine velocities
            ee_vel = np.hstack((lin_vel, ang_vel))

            # get joint velocities
            joint_vel = np.linalg.pinv(
                self._robot.jacob0(self._robot.q)) @ np.transpose(ee_vel)

            #@todo:
            # -- adding sigulariry check and joint limit check
            # -- consider adding collision check   
            
            # update joint states as a command to robot
            current_js = self._robot.q
            q = current_js + joint_vel * time_step
            self._robot.q = q

            index += 1
            if index == len(path)-1 and not is_arrived():
                print('Pose is unreachable!')
                break

            self._env.step(time_step)

    def go_to_joint_angles(self, q : np.ndarray, time=1, tolerance=0.0001):
            
        """
        ### Move robot to desired joint angles
        Function to move robot to desired joint angles. 
        This function is performing tehcnique called RMRC (Resolved Motion Rate Control) to move robot to desired pose.
        - @param q: desired joint angles           
        - @param time: time to complete the motion
        - @param tolerance: tolerance to consider the robot has reached the desired pose

        """

        step = 100
        time_step = time/step
        current_js = self._robot.q

        path = rtb.jtraj(current_js, q, t=step)

        # get ee carterian pose difference wih desired pose
        def is_arrived():
            js = self._robot.q
            poss_diff = np.diff(js - q)
            if np.all(np.abs(poss_diff) < tolerance):
                print('done')
                return True

            return False

        index = 0
        while not is_arrived():

            current_js = path[index]

    def rmrc_no_orientation(self, pose : sm.SE3, time=1, tolerance=0.0001):

        """
        ### Move robot to desired Cartesian pose
        Function to move robot end-effector to desired Cartesian pose. 
        This function is performing tehcnique called RMRC (Resolved Motion Rate Control) to move robot to desired pose.
        - @param pose: desired Cartesian pose           
        - @param time: time to complete the motion
        - @param tolerance: tolerance to consider the robot has reached the desired pose

        """

        step = 100
        time_step = time/step
        # current_ee_pose = self._robot.get_ee_pose()

        # get ee carterian pose difference wih desired pose
        def is_arrived():
            ee_pose = self._robot.fkine(self._robot.q)
            poss_diff = np.diff(ee_pose.A - pose.A)
            if np.all(np.abs(poss_diff) < tolerance):
                print('done')
                return True

            return False
        
        # get unit vector of velocity
        def get_unit_vector(vect):
            return vect / np.linalg.norm(vect)

        pose_threshold = 0.01
        linear_vel = 0.1
        kp = 1
        
        while not is_arrived():

            # get current ee pose
            current_ee_pose = self._robot.get_ee_pose()

            # get linear velocity between current pose and desired pose
            pos_diff = (pose.A[0:3, 3] - current_ee_pose.A[0:3, 3])
            unit_vel = get_unit_vector(pos_diff)/time_step

            if np.linalg.norm(pos_diff) < pose_threshold:
                linear_vel = kp * np.linalg.norm(pos_diff)

            xdot = unit_vel * linear_vel
            qdot = np.linalg.pinv(self._robot.jacob0(self._robot.q)[0:3,:]) @ np.transpose(xdot)

            # update joint states as a command to robot
            current_js = self._robot.q
            q = current_js + qdot * time_step
            self._robot.q = q

            self._env.step(time_step)

    def is_singulared(self):
        pass
