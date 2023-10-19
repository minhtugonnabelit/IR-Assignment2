from abc import abstractmethod, ABC
from swift import Swift
from robot.m_DHRobot3D import M_DHRobot3D
from controller import Controller
import logging

class ControllerInterface():

    def __init__(self, robot : M_DHRobot3D, env : Swift, log : logging,is_sim=True):
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

    # def update_collision_object(self, obj):
    #     """
    #     Update collision object
    #     """
    #     self._impl.update_collision_object(obj)


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

    def get_busy_status(self):
        return self._impl._get_busy_status()
        

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

