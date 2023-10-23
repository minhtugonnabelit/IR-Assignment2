# use for testing only, should separate into different files-----
from robot import Astorino
from robot import Sawyer
from swift import Swift 
#----------------------------------------------------------------

from plate import Plate
from work_cell import WorkCell

import os
import copy
import time

import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
import spatialmath.base as smb
import spatialgeometry as geometry
from controller_interface import ControllerInterface



class Mission():
    """
    TODO: Add a getter for current position of the controller in controller interface

    NOTE: 
        The Mission class is designed for general use of various robot controllers that adhere to 
        ControllerInterface module. Two robot platforms will be use: one robot is the picker and the 
        other is the bender. Implement with this sense of abstraction in mind and avoid specific naming

        The environment should be abstracted from this class, i.e don't mention Swift

    Future improvement: Operate on several printing machines, have a queue to store missions, each element is
        [int(printer_index), string(current_mission)]

    """
    def __init__(self, plate : Plate, workcell : WorkCell, picker_robot : ControllerInterface, bender_robot : ControllerInterface):
        

        # -----------------
        # Use getter function (TODO) of these controllers to keep track of their current position /
        self._plate = plate
        self._picker_robot = picker_robot
        self._bender_robot = bender_robot

        #----------------
        self._plate_position = sm.SE3(0.87,0.87,0.87)
        self._step = 10

    def _home_system(self):
        """
        STEP 1: Homing both robot arms, ready to pick
        """

        self._bender_robot.send_command('HOME')
        self._picker_robot.send_command('HOME')
    
    def _looking_for_plate(self):
        """
        Set the end effector pose looking 45 degrees downward
        Turns around until the finds the plate
        """
        
        # After found, set this variable
        self._plate_position = sm.SE3()
        pass

    def _grip_plate_edge(self):
        """
        STEP 2 : Grip the edge of the plate
        After this step, use the _plate_position to coordinate the arm(s)
        """
        pass

    def _move_plate(self, postion : sm.SE3):
        """
        STEP 3: Move the print plate to the specified position
        """
        pass

    def _astor_grip(self):
        """
        STEP 4: Astor joins by gripping the other side of the plate
        """
        pass

    def _drop_obejct(self):
        """
        STEP 5: Drop the object
        After this step, consider mission done, can send completion signal, move back home 
        or move on to another mission
        """
        self._tilt_plate()
        self._bend_plate()


    def _tilt_plate(self, forth = True):
        """
        Coordinate 2 arms to tilt the plate to a specified orientation to drop the object
        - @param forth: True for tilting forth, False for back
        """
        pass

    def _bend_plate(self):
        """
        Coordinate 2 arms to bend the plate and return to unbend position
        """
        pass


    # TAM ----------------------------------------------------------------------------
    def launch_system(self):
        """
        Launching both robot arms"""

        self._bender_robot.launch()
        self._picker_robot.launch()


    def enable_system(self):
        """
        Enable both robot arms
        """

        self._bender_robot.send_command('ENABLE')
        self._picker_robot.send_command('ENABLE')

        self._step ='SOMETHING'
    

    def stop_system(self):
        """
        Stop both robot arms
        """
        self._bender_robot.engage_estop()
        self._picker_robot.engage_estop()
        

    def update_collision_object(self, side, center):

        viz_object = self._picker_robot.update_collision_object(side, center)
        self._bender_robot.update_collision_object(side, center)

        return viz_object


    def run(self):
        print('bruh')
        qgoal_a = [0, 0, 0, 0, 0, 0]
        self._bender_robot.go_to_joint_angles(qgoal_a)


    def test(self):
        pass

        
if __name__ == "__main__":

    env = Swift()
    env.launch(realtime= True)
    
    sawyer_robot = Sawyer(env= env)
    sawyer_controller = ControllerInterface(sawyer_robot)
    
    astorino_robot = Astorino(env= env, base= sm.SE3(0,1,0))
    astorino_controller = ControllerInterface(astorino_robot)
    
    plate = Plate(sm.SE3(0,0,0), env)
    
    mission = Mission(plate, sawyer_controller, astorino_controller)
    mission.launch_system() # only called once when both controller havent been launched, otherwise, dont do this
    mission.enable_system() # only called when controllers need to be enabled
    mission._home_system()
