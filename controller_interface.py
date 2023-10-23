from abc import abstractmethod, ABC
from swift import Swift
from robot.m_DHRobot3D import M_DHRobot3D
from controller import Controller
import logging
import copy 

class ControllerInterface():

    def __init__(self, robot : M_DHRobot3D, env : Swift, log : logging, is_sim=True):
        self._impl = Controller(robot, env, log, is_sim)


    def launch(self):
        """disable_gamepad
        Start the controller
        """
        self._impl.launch()


    def system_activated(self):
        """
        Check if system is activated
        """
        return self._impl.system_activated()


    def update_collision_object(self, side, center):
        """
        Update collision object
        """
        return self._impl.update_collision_object(side, center)


    def gamepad_control(self):
        """
        ### Gamepad control
        Function to control robot using gamepad
        """
        self._impl.gamepad_control()


    def disable_gamepad(self):
        """
        ### Disable gamepad
        Function to disable gamepad control
        """
        self._impl.disable_gamepad()


    def send_command(self, command):
        """
        Send command to controller
        """
        self._impl.send_command(command)


    def shutdown(self):
        """
        Shutdown the controller 
        """
        self._impl.shutdown()


    def clean(self):
        """
        Clean up the controller
        """
        self._impl.clean()

    def go_to_home(self):
        """
        ### Move robot to home position
        Function to move robot to home position
        - @param time: time to complete the motion
        """
        self._impl.go_to_home()


    def set_joint_value(self, j, value):
        """
        Set joint value
        """
        self._impl.set_joint_value(j, value)


    def update_js(self):
        """
        Update joint states
        """
        self._impl._update_js()
    

    def update_robot_js(self):
        self._impl._update_robot_js()


    def move(self):
        """
        Execute a joint space trajectory
        """
        self._impl.move()


    def set_cartesian_value(self, pose):
        """
        Set cartesian value
        """
        self._impl.set_cartesian_value(pose)


    def move_cartesian(self):
        """
        Execute a cartesian space trajectory
        """
        self._impl.move_cartesian()


    def follow_cartesian_path(self, path, time=1, tolerance=0.001):
        """
        Follow a cartesian path
        """
        self._impl.follow_cartesian_path(path, time, tolerance)


    def go_to_cartesian_pose(self, pose, time=1, tolerance=0.001):
        """
        Go to cartesian pose
        """
        self._impl.go_to_cartesian_pose(pose, time, tolerance)
        

    def get_busy_status(self):
        return self._impl._get_busy_status()
        

    # Getters
    # -----------------------------------------------------------------------------------#
    def get_robot(self):
        """
        Getter for robot object """

        return self._impl._robot
    
    def get_env(self):
        """
        Getter for environment object """

        return self._impl._env
    
    def get_collision_object(self):
        """
        Getter for collision object """

        return self._impl.object
    
    def get_joint_angles(self):
        """
        Getter for joint angles """

        return copy.deepcopy(self._impl._robot.q)
    
    def get_ee_pose(self):
        """
        Getter for end-effector pose """
        
        return self._impl._robot.fkine(self._impl._robot.q)
    


    # STATE FUNCTION
    def system_state(self):
        """
        ### Get system state
        Function to get system state
        - @return: system state
        """
        return self._impl._state
    
    def engage_estop(self):
        self._impl.engage_estop()

    def update_estop_state(self):
        self._impl.update_estop_state()

    def disengage_estop(self):
        self._impl.disengage_estop()

    def enable_system(self):
        self._impl.enable_system()

    def disable_system(self):
        self._impl.disable_system()

