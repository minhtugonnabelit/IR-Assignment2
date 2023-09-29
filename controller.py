import copy
import sys
import pygame

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
        self._state = 'IDLE'
        self._joystick = self.joystick_init()
        self._joystick.init()

    def joystick_init(self):

        # Setup joystick
        pygame.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            raise Exception('No joystick found')
        else:
            joystick = pygame.joystick.Joystick(0)

        return joystick

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
        current_ee_pose = self._robot.fkine(self._robot.q)

        path = rtb.ctraj(current_ee_pose, pose, t=step)

        # get ee carterian pose difference wih desired pose
        def is_arrived():
            ee_pose = self._robot.fkine(self._robot.q)
            poss_diff = np.diff(ee_pose.A - pose.A)
            if np.all(np.abs(poss_diff) < tolerance):
                print('done')
                return True

            return False

        index = 0
        while not is_arrived():

            if self._state == "STOPPED":
                print("System is halted. Please disengage E-stop to continue.")
                break
            elif self._state == "IDLE":
                print("System is idle. Please enable to continue.")
                break                
    
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

            #@todo:
            # -- consider adding collision check   

            # calculate joint velocities
            joint_vel = self.solve_RMRC(ee_vel)
            
            # update joint states as a command to robot
            current_js = copy.deepcopy(self._robot.q)
            ee_height = self._robot.fkine(current_js).A[2, 3]
            q = current_js + joint_vel * time_step
            if abs(ee_height-self._robot.base.A[2,3]) < 0.01:
                q = self._robot.q
                print('ee is too close to the ground')
                break

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

            if self._state == "STOPPED":
                print("System is halted. Please disengage E-stop to continue.")
                break
            elif self._state == "IDLE":
                print("System is idle. Please enable to continue.")
                break
            
            self._robot.q = path.q[index]
            index += 1
            self._env.step(time_step)

    def gamepad_control(self,enable=True):

        # Print joystick information
        joy_name = self._joystick.get_name()
        joy_axes = self._joystick.get_numaxes()
        joy_buttons = self._joystick.get_numbuttons()

        print(f'Your joystick ({joy_name}) has:')
        print(f' - {joy_buttons} buttons')
        print(f' - {joy_axes} axes')

        vel_scale = {'linear': 0.3, 'angular': 0.8}

        # Main loop to check joystick functionality
        while enable:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            if self._joystick.get_button(4):
                self._robot.home()

            vz = 0
            if self._joystick.get_button(1):
                vz = 0.5
            elif self._joystick.get_button(2):
                vz = -0.5
            
            # get linear and angular velocity
            linear_vel = np.asarray([self._joystick.get_axis(1), -self._joystick.get_axis(0), vz]) * vel_scale['linear'] 
            angular_vel = np.asarray([self._joystick.get_axis(3), self._joystick.get_axis(4), 0]) * vel_scale['angular']

            # combine velocities
            ee_vel = np.hstack((linear_vel, angular_vel))

            # calculate joint velocities
            joint_vel = self.solve_RMRC(ee_vel)

            # update joint states as a command to robot
            current_js = copy.deepcopy(self._robot.q)
            q = current_js + joint_vel * 0.01
            self._robot.q = q

            self._env.step(0.01)

    def solve_RMRC(self, ee_vel):
        """
        ### Solve RMRC
        """
        # calculate manipulability
        j = self._robot.jacob0(self._robot.q)
        w = np.sqrt(np.linalg.det(j @ np.transpose(j)))

        # set threshold and damping
        w_thresh = 0.05
        damp = 0
        max_damp = 0.5

        # if manipulability is less than threshold, add damping
        if w < w_thresh:
            print(f'{self._robot.name} is near singularity!')
            damp = (1-np.power(w/w_thresh, 2)) * max_damp

        # calculate damped least square
        j_dls = j @ np.transpose(j) @ np.linalg.inv( j @ np.transpose(j) + damp * np.eye(6) )

        # get joint velocities, if robot is in singularity, use damped least square
        joint_vel = np.linalg.pinv(j) @ j_dls @ np.transpose(ee_vel)

        return joint_vel


    # SENSING FUNCTION
    # -----------------------------------------------------------------------------------#
    def system_state(self):
        """
        ### Get system state
        Function to get system state
        - @return: system state
        """
        return self._state

    def engage_estop(self):
        self._state = "STOPPED"
        print("E-stop engaged. System is halted.")

    def disengage_estop(self):
        if self._state == "STOPPED":
            self._state = "IDLE"
            print("E-stop disengaged. System is now idle and awaiting enable.")

    def enable_system(self):
        if self._state == "IDLE":
            self._state = "ENABLED"
            print("System is enabled. Ready for operation.")

    def disable_system(self):
        if self._state == "ENABLED":
            self._state = "IDLE"
            print("System is disabled. Back to idle state.")
    
