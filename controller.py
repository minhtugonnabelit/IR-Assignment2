import copy
import sys
import pygame
import queue
import time
import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
import spatialgeometry as geometry
from rectangularprism import RectangularPrism
from swift import Swift
from robot.m_DHRobot3D import M_DHRobot3D
from safety import Safety
import threading



class Controller():
    
    def __init__(self, robot : M_DHRobot3D, env : Swift, is_sim=True):

        self._robot = robot
        self._env   = env
        self._is_sim = is_sim
        self._state = 'IDLE'
        self._ui_js = np.zeros(np.size(self._robot.links))
        self._ui_pose = self._robot.fkine(self._ui_js)
        self._shutdown = False
        self._disable_gamepad = True
        self._command_queue = queue.Queue()
        self._safety = Safety(self._robot)

        # Dispatch table
        self._dispatch = {
            "ENABLE": self.enable_system,
            "HOME": self.go_to_home,
            "+X": lambda: self._go_to_CartesianPose(self._robot.fkine(self._robot.q) @ sm.SE3(0.05, 0, 0),),
            "-X": lambda: self._go_to_CartesianPose(self._robot.fkine(self._robot.q) @ sm.SE3(-0.05, 0, 0),),
            "+Y": lambda: self._go_to_CartesianPose(self._robot.fkine(self._robot.q) @ sm.SE3(0, 0.05, 0),),
            "-Y": lambda: self._go_to_CartesianPose(self._robot.fkine(self._robot.q) @ sm.SE3(0, -0.05, 0),),
            "+Z": lambda: self._go_to_CartesianPose(self._robot.fkine(self._robot.q) @ sm.SE3(0, 0, 0.05),),
            "-Z": lambda: self._go_to_CartesianPose(self._robot.fkine(self._robot.q) @ sm.SE3(0, 0, -0.05),),
            "+Rx": lambda: self._go_to_CartesianPose(self._robot.fkine(self._robot.q) @ sm.SE3.Rx(0.1), time=0.1),
            "-Rx": lambda: self._go_to_CartesianPose(self._robot.fkine(self._robot.q) @ sm.SE3.Rx(-0.1), time=0.1),
            "+Ry": lambda: self._go_to_CartesianPose(self._robot.fkine(self._robot.q) @ sm.SE3.Ry(0.1), time=0.1),
            "-Ry": lambda: self._go_to_CartesianPose(self._robot.fkine(self._robot.q) @ sm.SE3.Ry(-0.1), time=0.1),
            "+Rz": lambda: self._go_to_CartesianPose(self._robot.fkine(self._robot.q) @ sm.SE3.Rz(0.1), time=0.1),
            "-Rz": lambda: self._go_to_CartesianPose(self._robot.fkine(self._robot.q) @ sm.SE3.Rz(-0.1), time=0.1),
            "JOINT_ANGLES": self.move,
            "CARTESIAN_POSE": self.move_cartesian,
            "GAMEPAD_ENABLE": self.gamepad_control,
            "UPDATE_JOINT_STATE": self._update_js,
        }  

    
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
            return True
        else:
            print(f'System is not enabled. Current state: {self._state}')
            return False

    def update_collision_object(self, side, center):
        """
        Update collision object
        """
        # create RectangularPrism object for line_plane collision check
        self.object = RectangularPrism(width=side[0], breadth=side[1], height=side[2], center=center[0:3,3])
        self.vertices, self.faces, self.normals = self.object.get_data()

        # create Cuboid object for collision avoidance
        self.avoidance_object = geometry.Cuboid(side, pose=center)

        return self.avoidance_object

    # def update_collision_object(self, obj):
    #     """
    #     Update collision object
    #     """
        
    #     if isinstance(obj, RectangularPrism):
    #         self.object = obj
    #         self.vertices, self.faces, self.normals = obj.get_data()
    #     else: 
    #         self.object = obj

    def _joystick_init(self):

        # Setup joystick
        pygame.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            joystick = None
        else:
            joystick = pygame.joystick.Joystick(0)

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
                if command in self._dispatch:
                    # print(f'Executing command {command}')
                    self._dispatch[command]()
                else:
                    print(f'Command {command} is not recognized!')
            except queue.Empty:
                pass

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
        self._env.close()
        self.thread.join()

    def go_to_home(self):
        """
        ### Move robot to home position
        Function to move robot to home position
        - @param time: time to complete the motion
        """
        self.go_to_joint_angles(self._robot.neutral, time=2)


    # joint space interaction
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


    # cartesian space interaction
    def set_cartesian_value(self, pose):
        """
        Set cartesian value

        """
        self._ui_pose = pose
    
    def move_cartesian(self):
        """
        Execute a cartesian space trajectory

        """
        self._go_to_CartesianPose(self._ui_pose, time=3)

    def _update_js(self):
        self._robot.q = self._ui_js

    # --------------------------------------------------#
    # gamepad control
    def disable_gamepad(self):
        """
        ### Disable gamepad
        Function to disable gamepad control
        """
        self._disable_gamepad = True

    # def enable_gamepad(self):
    #     """
    #     ### Enable gamepad
    #     Function to enable gamepad control
    #     """
    #     self._disable_gamepad = False

    def gamepad_control(self):
        """
        ### Gamepad control
        Function to control robot using gamepad
        """

        self._joystick = self._joystick_init()
        self._disable_gamepad = False

        if self._joystick is not None:

            # Print joystick information
            self._joystick.init()
            joy_name = self._joystick.get_name()
            joy_axes = self._joystick.get_numaxes()
            joy_buttons = self._joystick.get_numbuttons()

            print(f'Your joystick ({joy_name}) has:')
            print(f' - {joy_buttons} buttons')
            print(f' - {joy_axes} axes')

            vel_scale = {'linear': 0.3, 'angular': 0.8}

            count = 0
            # Main loop to check joystick functionality
            while not self._disable_gamepad:

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        sys.exit()

                if self._joystick.get_button(3):
                    if self._state == 'STOPPED':
                        self.disengage_estop()
                    else:
                        self.engage_estop()

                if self._joystick.get_button(5):
                    self.enable_system()

                if self.system_activated():

                    if self._joystick.get_button(4):
                        self.go_to_home()

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

                    # collision avoidance damping
                    d_thresh = 0.01
                    time_step = 0.01
                    gamepad_gain = 0.8

                    # get distance between ee and object,  also extracting the closest points to use as damping velocity
                    d, p1, p2 = self._safety.collision_check(self._robot.q, self.object)
                    if d <= d_thresh:
                        vel = ( p1 - p2 ) / time_step
                        ee_vel[0:3] += gamepad_gain*vel

                    # calculate joint velocities
                    j = self._robot.jacob0(self._robot.q)
                    joint_vel = Controller.solve_RMRC(j, ee_vel)

                    # update joint states as a command to robot
                    current_js = copy.deepcopy(self._robot.q)
                    q = current_js + joint_vel * time_step

                    # check if ee is too close to the ground
                    if self._safety.grounded_check(q):
                        self._state = 'STOPPED'

                    if self._safety.is_self_collided(q):
                        self._state = 'STOPPED'


                    self._robot.q = q

                    self._env.step(time_step)
        else:
            print('No joystick found!')

    ### GENERAL MOTION FUNCTION
    # -----------------------------------------------------------------------------------#
    def is_arrived(self, pose : sm.SE3, tolerance=0.001):
        ee_pose = self._robot.fkine(self._robot.q)
        poss_diff = np.diff(ee_pose.A - pose.A)
        if np.all(np.abs(poss_diff) < tolerance):
            print('Provided goal is Done')
            return True

        return False
    
    def _go_to_CartesianPose(self, pose : sm.SE3, time=1, tolerance=0.001):
        
        """
        ### Move robot to desired Cartesian pose
        Function to move robot end-effector to desired Cartesian pose. 
        This function is performing tehcnique called RMRC (Resolved Motion Rate Control) to move robot to desired pose.
        - @param pose: desired Cartesian pose           
        - @param time: time to complete the motion
        - @param tolerance: tolerance to consider the robot has reached the desired pose

        """

        step = 50
        time_step = time/step
        current_ee_pose = self._robot.fkine(self._robot.q)

        path = rtb.ctraj(current_ee_pose, pose, t=step)

        vel_scale = {'linear': 0.6, 'angular': 0.8}

        index = 0
        while not self.is_arrived(pose,tolerance) and self.system_activated():
            
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
            s_omega = (path[index+1].A[0:3, 0:3] @ np.transpose(
                self._robot.fkine(self._robot.q).A[0:3, 0:3]) - np.eye(3)) / time_step
            ang_vel = [s_omega[2, 1], s_omega[0, 2], s_omega[1, 0]]

            # combine velocities
            ee_vel = np.hstack((lin_vel, ang_vel))
            
            ## CHEATING COLLISION AVOIDANCE METHODS ---------------------------------------------#
            self.avoidance = False
            if self._safety.collision_check_ee(self._robot.q, self.vertices, self.faces, self.normals):
                self.avoidance = True
                print('line_plane ee is nearly collided with object')

            if self.avoidance is True:
                # set threshold and damping
                d_thresh = 0.01

                # weight of the damping vel for collision avoidance
                weight = 0.1
                # get distance between ee and object,  also extracting the closest points to use as damping velocity
                d, p1, p2 = self._safety.collision_check(self._robot.q, self.avoidance_object)
                if d <= d_thresh:
                    vel = ( p1 - p2 ) / time_step
                    ee_vel[0:3] += weight*vel

            ## END -----------------------------------------------------------------------------#
            
            # calculate joint velocities, singularity check is already included, 
            # need to update for damping with self collision check 
            j = self._robot.jacob0(self._robot.q)
            joint_vel = Controller.solve_RMRC(j,ee_vel)
            
            # update joint states as a command to robot
            current_js = copy.deepcopy(self._robot.q)
            q = current_js + joint_vel * time_step

            # check if robot body is too close to the ground
            if self._safety.grounded_check(q):
                self._state = 'STOPPED'
            
            # check if ee is self collided
            if self._safety.is_self_collided(q):
                self._state = 'STOPPED'

            self._robot.q = q

            index += 1
            if index == len(path)-1 and not self.is_arrived(pose,tolerance):
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
        while not is_arrived() and self.system_activated():
            
            if isinstance(self.object, RectangularPrism):
                if self._safety.collision_check_ee(path.q[index], self.vertices, self.faces, self.normals):
                    print('line_plane ee is collided with object')
                    break

            else:
                d, p1, p2 = self._safety.collision_check(path.q[index], self.object)
                d_thresh = 0.01
                if d <= d_thresh:
                    print('ee is collided with object')
                    break


            # check if robot body is too close to the ground
            if self._safety.grounded_check(path.q[index]):
                self._state = 'STOPPED'
            
            # check if ee is self collided
            if self._safety.is_self_collided(path.q[index]):
                self._state = 'STOPPED'

            self._robot.q = path.q[index]
            index += 1
            self._env.step(time_step)

    @staticmethod
    def solve_RMRC(j, ee_vel):
        """
        ### Solve RMRC
        """
        # calculate manipulability
        w = np.sqrt(np.linalg.det(j @ np.transpose(j)))

        # set threshold and damping
        w_thresh = 0.04
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
    
    

    # STATE FUNCTION
    # -----------------------------------------------------------------------------------#
    def engage_estop(self):
        self._state = "STOPPED"
        print("E-stop engaged. System is halted.")

    def update_estop_state(self):
        if self._state == 'ENABLED' or self._state == 'IDLE':
            self._state = 'STOPPED'
        else:
            self._state = 'IDLE'

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

   
