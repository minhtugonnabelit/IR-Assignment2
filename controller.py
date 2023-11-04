import copy
import sys
import os
import pygame
import queue
import time
import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
import spatialgeometry as geometry
from rectangularprism import RectangularPrism
from robot.m_DHRobot3D import M_DHRobot3D
from safety import Safety

import threading
import logging
import time



class Controller():
    
    def __init__(self, robot : M_DHRobot3D, log: logging, is_sim=True):

        self._robot = robot
        self._is_sim = is_sim
        self._state = 'IDLE'
        self._ui_js = self._robot.q
        self._ui_pose = self._robot.fkine(self._ui_js)
        self._shutdown = False
        self._disable_gamepad = True
        self._gamepad_status = False
        self._command_queue = queue.Queue()
        self._safety = Safety(self._robot, log)
        self._robot_busy = False
        self.object = None
        self._log = log
        self.action_done = True
        self._print_once = True
        self._read_event = None
        self.is_collided = False
        
        if self._robot.name == 'Sawyer':
            self._ee_offset = sm.SE3(0, 0, 0.16)
        elif self._robot.name == 'Astorino':
            self._ee_offset = sm.SE3(0, 0, 0.1)

        # Dispatch table
        self._dispatch = {
            
            "ENABLE": self.enable_system,
            "DISABLE": self.disable_system,
            "HOME": self.go_to_home,
            "+X": lambda: self.go_to_cartesian_pose(self._robot.fkine(self._robot.q) @ sm.SE3(0.1, 0, 0),),
            "-X": lambda: self.go_to_cartesian_pose(self._robot.fkine(self._robot.q) @ sm.SE3(-0.1, 0, 0),),
            "+Y": lambda: self.go_to_cartesian_pose(self._robot.fkine(self._robot.q) @ sm.SE3(0, 0.1, 0),),
            "-Y": lambda: self.go_to_cartesian_pose(self._robot.fkine(self._robot.q) @ sm.SE3(0, -0.1, 0),),
            "+Z": lambda: self.go_to_cartesian_pose(self._robot.fkine(self._robot.q) @ sm.SE3(0, 0, 0.1),),
            "-Z": lambda: self.go_to_cartesian_pose(self._robot.fkine(self._robot.q) @ sm.SE3(0, 0, -0.1),),
            "+Rx": lambda: self.go_to_cartesian_pose(self._robot.fkine(self._robot.q) @ sm.SE3.Rx(0.1), duration=0.1),
            "-Rx": lambda: self.go_to_cartesian_pose(self._robot.fkine(self._robot.q) @ sm.SE3.Rx(-0.1), duration=0.1),
            "+Ry": lambda: self.go_to_cartesian_pose(self._robot.fkine(self._robot.q) @ sm.SE3.Ry(0.1), duration=0.1),
            "-Ry": lambda: self.go_to_cartesian_pose(self._robot.fkine(self._robot.q) @ sm.SE3.Ry(-0.1), duration=0.1),
            "+Rz": lambda: self.go_to_cartesian_pose(self._robot.fkine(self._robot.q) @ sm.SE3.Rz(0.1), duration=0.1),
            "-Rz": lambda: self.go_to_cartesian_pose(self._robot.fkine(self._robot.q) @ sm.SE3.Rz(-0.1), duration=0.1),

            "CARTESIAN_POSE": self.move_cartesian,
            "GAMEPAD_ENABLE": self.gamepad_control,
            "UPDATE_JOINT_STATE": self._update_js,

            "GO_TO_CARTESIAN_POSE": self.go_to_cartesian_pose,
            "GO_TO_JOINT_ANGLES": self.go_to_joint_angles,
        }  
        
        self.joystick_object = None

    
    def launch(self):
        """
        Start the controller
        """
        self.thread = threading.Thread(target=self._run)
        self.thread.start()
            
    def system_activated(self):
        """
        Check if system is activated
        """
        if self._state == "ENABLED":
            self._print_once = True
            return True
        else:
            if self._print_once:
                print(f'System is not enabled. Current state: {self._state}')
                self._print_once = False 
            return False

    def update_collision_object(self, side, center):
        """
        Update collision object
        """
        # create RectangularPrism object for line_plane collision check
        self.object = RectangularPrism(width=side[0], breadth=side[1], height=side[2], center=center.A[0:3,3])
        self.vertices, self.faces, self.normals = self.object.get_data()

        # create Cuboid object for collision avoidance
        self.avoidance_object = geometry.Cuboid(side, pose=center, color= (0.549,0.29,0.004,0.7))

        return self.avoidance_object

    @staticmethod
    def _joystick_init(self):

        # Setup joystick
        pygame.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            joystick = None
        else:
            joystick = pygame.joystick.Joystick(0)
            self.joystick_object = joystick

        return joystick
    
    ### GUI FUNCTION
    # -----------------------------------------------------------------------------------#
    def _run(self):
        """
        Run the controller
        """
        while not self._shutdown:
            try: 
                command = self._command_queue.get(timeout=0.01)

                if isinstance(command, str):
                    command_name = command
                    command_args = []
                else:
                    command_name = command["name"]
                    command_args = command.get("args", [])

                if command_name in self._dispatch:
                    self._dispatch[command_name](*command_args)
                else:
                    print(f'Command {command_name} is not recognized!')
                    
            except queue.Empty:
                pass

            # time.sleep(0.5)

    def send_command(self, command):
        """
        Send command to controller
        """
        self._command_queue.put(command)

    def shutdown(self):
        """
        Shutdown the controller 
        """
        self._shutdown = True  
    
    def clean(self):
        """
        Clean up the controller
        """
        self.shutdown()
        self.thread.join()

    def go_to_home(self):
        """
        ### Move robot to home position
        Function to move robot to home position
        - @param time: time to complete the motion
        """
        self.go_to_joint_angles(self._robot.neutral, duration=2)


    # -----------------------------------------------------------------------------------#
    # Joint space interaction

    def set_joint_value(self, j, value):
        """
        Set joint value

        """
        self._ui_js[j] = np.deg2rad(float(value))

    def move(self):
        """
        Execute a joint space trajectory

        """
        self.go_to_joint_angles(self._ui_js, time=3)

    def _update_js(self):
        """
        """

        if self._safety.grounded_check(self._ui_js) or self._safety.is_self_collided(self._ui_js):
            self._state = 'STOPPED'
        else:
            self._robot.send_joint_command(self._ui_js)

    def _update_robot_js(self):
        self._ui_js = self._robot.q


    # -----------------------------------------------------------------------------------#
    # Cartesian space interaction

    def set_cartesian_value(self, pose):
        """
        Set cartesian value

        """
        self._ui_pose = pose
    
    def move_cartesian(self):
        """
        Execute a cartesian space trajectory

        """
        self.go_to_cartesian_pose(self._ui_pose, duration=2)

    # -----------------------------------------------------------------------------------#
    # Gripper function:

    def open_gripper(self):
        """
        Open gripper
        """
        self._robot.open_gripper()


    def close_gripper(self):
        """
        Close gripper
        """
        self._robot.close_gripper()

        
    # -----------------------------------------------------------------------------------#
    # gamepad control
    def disable_gamepad(self):
        """
        ### Disable gamepad
        Function to disable gamepad control
        """
        self._gamepad_status = False
        self._disable_gamepad = True

    def get_gamepad_status(self):
        return self._gamepad_status

    def gamepad_control(self):
        """
        ### Gamepad control
        Function to control robot using gamepad
        This function extract value of each axis and button from gamepad and 
        convert it to robot end effector velocity to solve for joint velocity
        """
        self.joystick_object = Controller._joystick_init(self)
 
        self._disable_gamepad = False
        self._gamepad_status = True
        time_step = 0.001

        # if joystick is not None:
        if self.joystick_object is not None:
            
            # Print joystick information

            # joystick.init()
            self.joystick_object.init()
            vel_scale = {'linear': 10, 'angular': 20}


            # estop count
            last_estop_button = False


            # Main loop to check joystick functionality

            if os.name == 'nt': # Windows
                self._gamepad_windows(joystick= self.joystick_object, # joystick
                                    vel_scale=vel_scale,
                                    time_step=time_step)
                
                
            else: # Linux
                self._gamepad_linux(joystick= self.joystick_object,
                                    vel_scale=vel_scale,
                                    last_estop_button= last_estop_button,
                                    time_step=time_step)
                
            
        else:
            self._log.error('No joystick found!')

    def get_gamepad_name(self):
        self.joystick_object = Controller._joystick_init(self)
        return self.joystick_object.get_name()

    def _gamepad_windows(self, joystick, vel_scale, time_step):
        while not self._disable_gamepad:
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            # estop ------------------------------------
            estop_button = joystick.get_button(2)
            release_estop_button = joystick.get_button(3)
            
            if estop_button and self._state == 'ENABLED':
                self.engage_estop()
            elif release_estop_button:
                if self._state == 'STOPPED':
                    self.disengage_estop()  
            
            # -------------------------------------------

            if joystick.get_button(10) or self._read_event == '-ENABLE-' or self._read_event == '-A_ENABLE-':
                self.enable_system()
                self._read_event = None

            if self.system_activated():

                if joystick.get_button(8) and joystick.get_button(9) or self._read_event == '-HOME-' or self._read_event == '-A_HOME-':
                    
                    self.go_to_home()

                if joystick.get_button(5):
                    self._robot.open_gripper()

                if joystick.get_button(4):
                    self._robot.close_gripper()
                
                
                # Filter
                axes_threshold = 0.2
                gamepad_max_gain = 1
                gamepad_min_gain = 0
                linear_vel_z = 0
                angular_vel_z = 0
                
                # Get the joystick axes
                if abs(joystick.get_axis(0)) >= axes_threshold:  # Control Linear X
                    linear_vel_x = joystick.get_axis(0)
                    if linear_vel_x > 0: linear_vel_x = np.interp(linear_vel_x, [axes_threshold, gamepad_max_gain], [gamepad_min_gain ,gamepad_max_gain])
                    else: linear_vel_x = np.interp(linear_vel_x, [-gamepad_max_gain, -axes_threshold], [-gamepad_max_gain ,-gamepad_min_gain])
                else: linear_vel_x = 0
        
                
                # BECAUSE WE WANT TO UTILIZE AXIS 1 (LEFT JOYSTICK : MOVE DOWNWARD AND UPWARD)
                
                if joystick.get_button(0):  # Control Linear Z
                    if abs(joystick.get_axis(1)) >= axes_threshold: 
                        linear_vel_z = -joystick.get_axis(1)
                        if linear_vel_z > 0: linear_vel_z = np.interp(linear_vel_z, [axes_threshold, gamepad_max_gain], [gamepad_min_gain ,gamepad_max_gain])
                        else: linear_vel_z = np.interp(linear_vel_z, [-gamepad_max_gain, -axes_threshold], [-gamepad_max_gain ,-gamepad_min_gain])
                    else: linear_vel_z = 0   
                else:                        
                    if joystick.get_button(1):   # Control Angular Z
                        if abs(joystick.get_axis(1)) >= axes_threshold: 
                            angular_vel_z = -joystick.get_axis(1)
                            if angular_vel_z > 0: angular_vel_z = np.interp(angular_vel_z, [axes_threshold, gamepad_max_gain], [gamepad_min_gain ,gamepad_max_gain])
                            else: angular_vel_z = np.interp(angular_vel_z, [-gamepad_max_gain, -axes_threshold], [-gamepad_max_gain ,-gamepad_min_gain])
                        else: angular_vel_z = 0 
                        
                    else:   # Control Linear Y
                        if abs(joystick.get_axis(1)) >= axes_threshold: 
                            linear_vel_y = -joystick.get_axis(1)
                            if linear_vel_y > 0: linear_vel_y = np.interp(linear_vel_y, [axes_threshold, gamepad_max_gain], [gamepad_min_gain ,gamepad_max_gain])
                            else: linear_vel_y = np.interp(linear_vel_y, [-gamepad_max_gain, -axes_threshold], [-gamepad_max_gain ,-gamepad_min_gain])
                        else: linear_vel_y = 0    
                    

                if abs(joystick.get_axis(2)) >= axes_threshold:     # Control Angular X
                    angular_vel_x = joystick.get_axis(2)
                    if angular_vel_x > 0: angular_vel_x = np.interp(angular_vel_x, [axes_threshold, gamepad_max_gain], [gamepad_min_gain ,gamepad_max_gain])
                    else: angular_vel_x = np.interp(angular_vel_x, [-gamepad_max_gain, -axes_threshold], [-gamepad_max_gain ,-gamepad_min_gain])
                else: angular_vel_x = 0
            
                if abs(joystick.get_axis(3)) >= axes_threshold:     # Control Angular Y
                    angular_vel_y = joystick.get_axis(3)
                    if angular_vel_y > 0: angular_vel_y = np.interp(angular_vel_y, [axes_threshold, gamepad_max_gain], [gamepad_min_gain ,gamepad_max_gain])
                    else: angular_vel_y = np.interp(angular_vel_y, [-gamepad_max_gain, -axes_threshold], [-gamepad_max_gain ,-gamepad_min_gain])
                else: angular_vel_y = 0    
                
    
                linear_vel = np.asarray([linear_vel_x, linear_vel_y, linear_vel_z]) * vel_scale['linear'] 
                angular_vel = np.asarray([angular_vel_x, angular_vel_y, angular_vel_z]) * vel_scale['angular']                
                ee_vel = np.hstack((linear_vel, angular_vel))
               
               
                # collision avoidance damping
                d_thresh = 0.05

                # Check collision
                self.is_collided = False
                if self.object is not None:
                    if self._safety.collision_check_ee(self._robot.q, self.vertices, self.faces, self.normals, threshold=0.0):
                        self.is_collided = True
                        # self._log.warning('line_plane ee is nearly collided with object')
                        
                    if self.is_collided is True:
                        # get distance between ee and object,  also extracting the closest points to use as damping velocity
                        d, p1, p2 = self._safety.collision_check(self._robot.q, self.avoidance_object)
                        if d <= d_thresh:
                            vel = ( p1 - p2 ) / time_step
                            ee_vel[0:3] += gamepad_max_gain * vel


                # calculate joint velocities

                j = self._robot.jacob0(self._robot.q)
                mu_threshold = 0.04 if self._robot._name == "Sawyer" else 0.01

                joint_vel = Controller.solve_RMRC(j, ee_vel, mu_threshold=mu_threshold)


                # update joint states as a command to robot

                current_js = copy.deepcopy(self._robot.q)
                q = current_js + joint_vel * time_step


                # check if ee is too close to the ground

                if self._safety.grounded_check(q) or self._safety.is_self_collided(q):
                    self._state = 'STOPPED'
                    continue

                # send joint command to robot to execute desired motion. Currently available mode is position mode
                self._robot.send_joint_command(q)
                
            time.sleep(0.01)


    # ------------------------------- GAMEPAD RUN LINUX
    
    def _gamepad_linux(self, joystick, vel_scale, last_estop_button, time_step):
        while not self._disable_gamepad:

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            # estop ------------------------------------
            estop_button = joystick.get_button(3)

            if estop_button:      
                if not last_estop_button:
                    last_estop_button = True
                    if self._state == 'STOPPED':
                        self.disengage_estop()
                    else:
                        self.engage_estop()
            else:
                last_estop_button = False
            # -------------------------------------------

            if joystick.get_button(5):
                self.enable_system()

            if self.system_activated():

                if joystick.get_button(4):
                    self.go_to_home()

                if joystick.get_button(6):
                    self._robot.open_gripper()

                if joystick.get_button(7):
                    self._robot.close_gripper()

                vz = 0
                if joystick.get_button(1):
                    vz = 0.5
                elif joystick.get_button(2):
                    vz = -0.5
                

                # get linear and angular velocity

                linear_vel = np.asarray([joystick.get_axis(1), -joystick.get_axis(0), vz]) * vel_scale['linear'] 
                angular_vel = np.asarray([0, joystick.get_axis(4), joystick.get_axis(3)]) * vel_scale['angular']
                ee_vel = np.hstack((linear_vel, angular_vel))


                # collision avoidance damping

                d_thresh = 0.05
                
                gamepad_max_gain = 1

                self.is_collided = False
                if self.object is not None:
                    if self._safety.collision_check_ee(self._robot.q, self.vertices, self.faces, self.normals, threshold=0.0):
                        self.is_collided = True
                        # self._log.warning('line_plane ee is nearly collided with object')
                        
                    if self.is_collided is True:
                        # get distance between ee and object,  also extracting the closest points to use as damping velocity
                        d, p1, p2 = self._safety.collision_check(self._robot.q, self.avoidance_object)
                        if d <= d_thresh:
                            vel = ( p1 - p2 ) / time_step
                            ee_vel[0:3] += gamepad_max_gain * vel


                # calculate joint velocities

                j = self._robot.jacob0(self._robot.q)
                mu_threshold = 0.04 if self._robot._name == "Sawyer" else 0.01

                joint_vel = Controller.solve_RMRC(j, ee_vel, mu_threshold=mu_threshold)


                # update joint states as a command to robot

                current_js = copy.deepcopy(self._robot.q)
                q = current_js + joint_vel * time_step


                # check if ee is too close to the ground

                if self._safety.grounded_check(q) or self._safety.is_self_collided(q):
                    self._state = 'STOPPED'
                    continue

                # send joint command to robot to execute desired motion. Currently available mode is position mode
                self._robot.send_joint_command(q)
                
            time.sleep(time_step)

    # -----------------------------------------------------------------------------------#
    ### GENERAL MOTION FUNCTION

    def is_arrived(self, pose : sm.SE3, tolerance=0.001):
        """
        Function to check if robot is arrived at desired pose
        - @param pose: desired pose
        - @param tolerance: tolerance to consider the robot has reached the desired pose
        """

        ee_pose = self._robot.fkine(self._robot.q)
        poss_diff = np.diff(ee_pose.A - pose.A)
        if np.all(np.abs(poss_diff) < tolerance):
            # self._log.info('Provided goal is Done')
            self.action_done = True
            return True
        self.action_done = False
        return False
    
    
    def _get_busy_status(self):
        return self._robot_busy


    def robot_is_collided(self):
        """
        Function to return if robot is collided. 
        Possible to be used to trigger avoidance motion
        """

        return self.is_collided


    def follow_cartesian_path(self, path, duration=1, tolerance=0.001):
        """
        ### Move robot to follow desired Cartesian path
        Function to move robot end-effector to follow desired Cartesian path.
        This function is performing tehcnique called RMRC (Resolved Motion Rate Control) to move robot to desired pose.
        - @param path: desired Cartesian path (a list or numpy array of SE3 pose)
        - @param duration: time to complete the motion (default 1 second)
        - @param tolerance: tolerance to consider the robot has reached the desired pose (default 0.001)
        """

        time_step = 1/50
        index = 0
        pose = path[-1]
        while index < len(path) and self.system_activated():

            self._robot_busy = True
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


            ## CHEATING COLLISION AVOIDANCE METHODS ---------------------------------------------#
            
            self.is_collided = False
            if self.object is not None:
                if self._safety.collision_check_ee(self._robot.q, self.vertices, self.faces, self.normals):
                    self.is_collided = True
                    self._log.warning('line_plane ee is nearly collided with object')
                    # break

                if self.is_collided is True:

                    # set threshold and damping
                    
                    d_thresh = 0.01


                    # weight of the damping vel for collision avoidance
                    
                    weight = 0.5


                    # get distance between ee and object,  also extracting the closest points to use as damping velocity
                    
                    d, p1, p2 = self._safety.collision_check(self._robot.q, self.avoidance_object)
                    if d <= d_thresh:
                        vel = ( p1 - p2 ) / time_step
                        ee_vel[0:3] += weight*vel


            ## END -----------------------------------------------------------------------------#
            
            # calculate joint velocities, singularity check is already included in the function
            
            j = self._robot.jacob0(self._robot.q)
            mu_threshold = 0.04 if self._robot._name == "Sawyer" else 0.01

            joint_vel = Controller.solve_RMRC(j,ee_vel,mu_threshold=mu_threshold)
            

            # update joint states as a command to robot
           
            current_js = copy.deepcopy(self._robot.q)
            q = current_js + joint_vel * time_step


            # check safety functionality before sending to execute
            
            if self._safety.grounded_check(q) or self._safety.is_self_collided(q):
                self._state = 'STOPPED'
                break

            index += 1
            if index == len(path)-1 and not self.is_arrived(pose,tolerance):
                self._log.info('Pose is unreachable!')
                break


            # send joint command to robot to execute desired motion. Currently available mode is position mode
            
            self._robot.send_joint_command(q)
            time.sleep(time_step)


        self._robot_busy = False

    
    def go_to_cartesian_pose(self, pose : sm.SE3, duration=1, tolerance=0.001):
        
        """
        ### Move robot to desired Cartesian pose
        Function to move robot end-effector to desired Cartesian pose. 
        This function is performing tehcnique called RMRC (Resolved Motion Rate Control) to move robot to desired pose.
        - @param pose: desired Cartesian pose           
        - @param duration: time to complete the motion
        - @param tolerance: tolerance to consider the robot has reached the desired pose
        """

        step = 50
        time_step = duration/step
        
        path = rtb.ctraj(self._robot.fkine(self._robot.q), pose, t=step)

        vel_scale = {'linear': 0.6, 'angular': 0.8}

        index = 0
        while not self.is_arrived(pose,tolerance) and self.system_activated():

            self._robot_busy = True

            # # Direction methods
            # # extracting linear vel direction
            # ee_cur_pose = self._robot.fkine(self._robot.q)
            # distance = np.linalg.norm(pose.A[0:3, 3] - ee_cur_pose.A[0:3, 3])
            # if np.isnan(distance) or distance < 0.005:
            #     lin_uint = np.array([0.0, 0.0, 0.0])
            #     distance = float(0.0)
            # else:
            #     lin_uint = (pose.A[0:3, 3] - ee_cur_pose.A[0:3, 3]) / distance
            # # extracting angular vel direction
            # s_omega = (pose.A[0:3, 0:3] @ np.transpose(
            #     ee_cur_pose.A[0:3, 0:3]) - np.eye(3))
            # orientation_error = np.linalg.norm([s_omega[2, 1], s_omega[0, 2], s_omega[1, 0]])
            # ang_unit = np.array([s_omega[2, 1], s_omega[0, 2], s_omega[1, 0]]) / orientation_error
            # # brake_thresh = 0.01
            # # ______________________________________
            # # missing a orientation brake threshold
            # # ___________________________________
            # # if distance < brake_thresh or orientation_error < brake_thresh:
            # #     linear_vel = lin_uint * vel_scale['linear'] * (distance / brake_thresh)**2
            # #     angular_vel = ang_unit * vel_scale['linear'] * (orientation_error / brake_thresh)**2
            # # else:
            # linear_vel = lin_uint * vel_scale['linear']
            # angular_vel = ang_unit * 2

            # ee_vel = np.hstack((linear_vel, angular_vel))
            # print(ee_vel)

            ## Linear interpolation methods
            # get linear velocity between interpolated point and current pose of ee
            
            prev_ee_pos = path[index].A[0:3, 3]
            desired_ee_pos = path[index+1].A[0:3, 3]


            # get linear velocity between interpolated point and current pose of ee
            
            lin_vel = (desired_ee_pos - prev_ee_pos) / time_step


            # get angular velocity between interpolated ...
            
            s_omega = (path[index+1].A[0:3, 0:3] @ np.transpose(self._robot.fkine(self._robot.q).A[0:3, 0:3]) - np.eye(3)) / time_step
            ang_vel = [s_omega[2, 1], s_omega[0, 2], s_omega[1, 0]]

            
            # combine velocities
            
            ee_vel = np.hstack((lin_vel, ang_vel))
            

            ## CHEATING COLLISION AVOIDANCE METHODS ---------------------------------------------#
            self.is_collided = False
            if self.object is not None:
                if self._safety.collision_check_ee(self._robot.q, self.vertices, self.faces, self.normals):
                    self.is_collided = True
                    self._log.warning('line_plane ee is nearly collided with object')
                    # break

                if self.is_collided is True:

                    # set threshold and damping
                    d_thresh = 0.01

                    # weight of the damping vel for collision avoidance
                    weight = 0.5

                    # get distance between ee and object,  also extracting the closest points to use as damping velocity
                    d, p1, p2 = self._safety.collision_check(self._robot.q, self.avoidance_object)
                    if d <= d_thresh:
                        vel = ( p1 - p2 ) / time_step
                        ee_vel[0:3] += weight*vel

            ## END -----------------------------------------------------------------------------#
            
            # calculate joint velocities, singularity check is already included in the function

            j = self._robot.jacob0(self._robot.q)
            mu_threshold = 0.04 if self._robot._name == "Sawyer" else 0.01
            joint_vel = Controller.solve_RMRC(j,ee_vel,mu_threshold=mu_threshold)
            

            # update joint states as a command to robot

            current_js = copy.deepcopy(self._robot.q)
            q = current_js + joint_vel * time_step


            # check safety functionality before sending to execute

            if self._safety.grounded_check(q) or self._safety.is_self_collided(q):
                self._state = 'STOPPED'
                break

            index += 1
            if index == len(path)-1 and not self.is_arrived(pose,tolerance):
                self._log.info('Pose is unreachable!')
                break


            # send joint command to robot to execute desired motion. Currently available mode is position mode

            self._robot.send_joint_command(q)
            # time.sleep(time_step)


        self._robot_busy = False
    
    def single_step_cartesian(self, pose : sm.SE3, time_step, tolerance=0.001):
        """
        Moves the robot end-effector to a desired pose in Cartesian space using a single step.

        Args:
            pose (sm.SE3): The desired pose of the end-effector.
            time_step (float): The time step for the motion.
            tolerance (float, optional): The tolerance for the motion. Defaults to 0.001.

        Returns:
            None
        """
        
        # self._robot_busy = True
        
        prev_ee_pos = self._robot.fkine(self._robot.q).A[0:3, 3]
        desired_ee_pos = pose.A[0:3, 3]


        # get linear velocity between interpolated point and current pose of ee
        
        lin_vel = (desired_ee_pos - prev_ee_pos) / time_step


        # get angular velocity between interpolated ...
        
        s_omega = (pose.A[0:3, 0:3] @ np.transpose(self._robot.fkine(self._robot.q).A[0:3, 0:3]) - np.eye(3)) / time_step
        ang_vel = [s_omega[2, 1], s_omega[0, 2], s_omega[1, 0]]

        
        # combine velocities
        
        ee_vel = np.hstack((lin_vel, ang_vel))
        

        ## CHEATING COLLISION AVOIDANCE METHODS ---------------------------------------------#
        self.is_collided = False
        if self.object is not None:
            if self._safety.collision_check_ee(self._robot.q, self.vertices, self.faces, self.normals):
                self.is_collided = True
                self._log.warning('line_plane ee is nearly collided with object')
                # break

            # if self.is_collided is True:

            #     # set threshold and damping
            #     d_thresh = 0.01

            #     # weight of the damping vel for collision avoidance
            #     weight = 0.5

            #     # get distance between ee and object,  also extracting the closest points to use as damping velocity
            #     d, p1, p2 = self._safety.collision_check(self._robot.q, self.avoidance_object)
            #     if d <= d_thresh:
            #         vel = ( p1 - p2 ) / time_step
            #         ee_vel[0:3] += weight*vel

        ## END -----------------------------------------------------------------------------#
        
        # calculate joint velocities, singularity check is already included in the function 

        j = self._robot.jacob0(self._robot.q)
        mu_threshold = 0.04 if self._robot._name == "Sawyer" else 0.01
        joint_vel = Controller.solve_RMRC(j,ee_vel,mu_threshold=mu_threshold)
        

        # update joint states as a command to robot

        current_js = copy.deepcopy(self._robot.q)
        q = current_js + joint_vel * time_step


        # check safety functionality before sending to execute

        if self._safety.grounded_check(q) or self._safety.is_self_collided(q):
            self._state = 'STOPPED'


        # send joint command to robot to execute desired motion. Currently available mode is position mode

        self._robot.send_joint_command(q)
        

    def single_step_joint(self, q : np.ndarray, time_step, tolerance=0.0001):

        self._robot_busy = True
        if self.object is not None:
            if self._safety.collision_check_ee(q, self.vertices, self.faces, self.normals):
                self._log.warning('line_plane ee is nearly collided with object')
                # self._state = 'STOPPED'
                self.is_collided = True

        if self._safety.grounded_check(q) or self._safety.is_self_collided(q):
            self._state = 'STOPPED'

        self._robot.send_joint_command(q)


    def go_to_joint_angles(self, q : np.ndarray, duration=1, tolerance=0.0001):
            
        """
        ### Move robot to desired joint angles
        Function to move robot to desired joint angles. 
        - @param q: desired joint angles (1xn numpy array)          
        - @param duration: time to complete the motion (default 1 second)
        - @param tolerance: tolerance to consider the robot has reached the desired pose (default 0.001)
        """

        step = 150
        time_step = duration/step

        # Defined quintic polynomial trajectory

        path = rtb.jtraj(self._robot.q, q, t=step)


        def is_arrived():
            js = copy.deepcopy(self._robot.q)
            poss_diff = np.diff(js - q)
            if np.all(np.abs(poss_diff) < tolerance):
                self._log.info('Provided joint states is Done')
                self.action_done = True
                return True
            self.action_done = False
            return False

        
        index = 0
        while not is_arrived() and self.system_activated():
            
            if self._gamepad_status: # Estop for homing in gamepad mode
                if self.joystick_object.get_button(2):
                    self._state = 'STOPPED'
                    break
                
            self._robot_busy = True
            if self.object is not None:
                if self._safety.collision_check_ee(path.q[index], self.vertices, self.faces, self.normals):
                    self._log.warning('line_plane ee is nearly collided with object')
                    self._state = 'STOPPED'
                    break

            # check if robot body is too close to the ground

            if self._safety.grounded_check(path.q[index]) or self._safety.is_self_collided(path.q[index]):
                self._state = 'STOPPED'
                break

            index += 1

            # send joint command to robot to execute desired motion. Currently available mode is position mode

            self._robot.send_joint_command(path.q[index])
            # time.sleep(0)


        self._robot_busy = False

    @staticmethod
    def solve_RMRC(j, ee_vel, mu_threshold=0.04):
        """
        ### Solve RMRC with given jacobian and desired ee velocity 
        - @param j: jacobian matrix (6xn numpy array)
        - @param ee_vel: desired ee velocity (6x1 vector) in format of [x,y,z,wx,wy,wz]
        - @param mu_threshold: manipulability threshold to determine if robot is in singularity. This threshhold varies for each type of robot.
        """

        # calculate manipulability

        w = np.sqrt(np.linalg.det(j @ np.transpose(j)))


        # set threshold and damping

        w_thresh = mu_threshold
        max_damp = 0.5


        # if manipulability is less than threshold, add damping

        if w < w_thresh:
            damp = (1-np.power(w/w_thresh, 2)) * max_damp 
        else: 
            damp = 0


        # calculate damped least square

        j_dls = j @ np.transpose(j) @ np.linalg.inv( j @ np.transpose(j) + damp * np.eye(6) )


        # get joint velocities, if robot is in singularity, use damped least square

        joint_vel = np.linalg.pinv(j) @ j_dls @ np.transpose(ee_vel)


        return joint_vel
    
    def action_is_done(self):
        return self.action_done

    # STATE FUNCTION
    # -----------------------------------------------------------------------------------#
    def engage_estop(self):
        self._state = "STOPPED"
        self._log.info("E-stop engaged. System is halted.")

    def update_estop_state(self):
        if self._state == 'ENABLED':
            self._state = 'STOPPED'
        else:
            self._state = 'IDLE'

    def disengage_estop(self):
        if self._state == "STOPPED":
            self._state = "IDLE"
            self._log.info("E-stop disengaged. System is now idle and awaiting enable.")

    def enable_system(self):
        if self._state == "IDLE":
            self._state = "ENABLED"
            self._log.info("System is enabled. Ready for operation.")

    def disable_system(self):
        if self._state == "ENABLED":
            self._state = "IDLE"
            self._log.info("System is disabled. Back to idle state.")

    def set_event_GUI(self, event: str):
        self._read_event = event
