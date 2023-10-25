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
        self._workcell = workcell
        self._picker_robot = picker_robot
        self._bender_robot = bender_robot

        #----------------
        self._cell_center = self._workcell.get_cell_center()
        self._cart_location = self._workcell.get_cart_location()
        # self._plate_position = self._plate.get_pose()
        self._step = 10

        # index to track current step
        self.index = 0

        self._action_list = [
            self._home_system,
            self._looking_for_plate,
            self._grip_plate_edge,
            self._lift_plate,
            self._hang_plate,
            self._move_plate,
            self._astor_grip,
            self._tilt_plate,
            self._bend_plate,
            self._drop_obejct,
            self._unbend_plate,
            self._return_plate,
            self._hold_plate,

        ]

    def _home_system(self):
        """
        STEP 1: Homing both robot arms, ready to pick
        """
        # non-blocking method
        self._bender_robot.send_command('HOME')
        self._picker_robot.send_command('HOME')
        print('system homed')
    
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

        ## Approach the plate and grip 
        """
        Supposed to get plate location and orientation from vision system
        OR directly get plate pose from object and use it to define grasp pose
        
        Both method currently not available, so use a fixed pose for now"""
                                    
        plate_pose = self._plate.get_pose()
        grip_pose = plate_pose @ sm.SE3(0.22,0,0) @ sm.SE3.RPY(0,-90,-180, unit = 'deg', order='xyz') 

        # grip_pose = sm.SE3(-0.09,0.91,0.91) 
        # non-blocking method
        command = {
            "name": "GO_TO_CARTESIAN_POSE",
            "args": [grip_pose]
        }
        self._picker_robot.send_command(command)

        # # blocking method
        # self._picker_robot.go_to_cartesian_pose(grip_pose)

        print('plate gripped')


    def _lift_plate(self):
        """
        Lift the plate to a safe height
        """

        # close gripper
        self._picker_robot.close_gripper()
        time.sleep(0.5)

        lift_pose = sm.SE3(0,0,0.1) @ self._picker_robot.get_ee_pose()
        plate_pose = self._plate.get_pose()

        # non-blocking method
        command = {
            "name": "GO_TO_CARTESIAN_POSE",
            "args": [lift_pose]
        }
        self._picker_robot.send_command(command)

        for i in range(10):

            self._plate.move_flat_plate(plate_pose @ sm.SE3(0,0,0.01))
            time.sleep(0.01)

        # self._plate.move_flat_plate()

        # # blocking method
        # self._picker_robot.go_to_cartesian_pose(lift_pose)
        print('plate lifted')


    def _hang_plate(self):
        """
        Hang the plate to the printer
        """
        hang_pose = sm.SE3(0.1,0,0) @ self._picker_robot.get_ee_pose()

        # non-blocking method
        command = {
            "name": "GO_TO_CARTESIAN_POSE",
            "args": [hang_pose]
        }
        self._picker_robot.send_command(command)

        # # blocking method
        # self._picker_robot.go_to_cartesian_pose(hang_pose)
        print('plate hanged')


    def _move_plate(self):
        """
        ### STEP 3: 
        Move the print plate to the specified position
       
        manually generate a path other than a straight line to avoid singularity 
        OR use the feedback control to auto complete the path"""
        position = sm.SE3(0.51, 0.15, 0.87) @ sm.SE3.RPY(-90.0,0.6,90.0, unit = 'deg', order='xyz')

        plate_pose = sm.SE3(0,0.11,0) @ position 
        desired_js = self._picker_robot.get_robot().ikine_LM(plate_pose, q0 = self._picker_robot.get_joint_angles()).q

        # non-blocking method
        command = {
            "name": "GO_TO_JOINT_ANGLES",
            "args": [desired_js]
        }
        self._picker_robot.send_command(command)

        # # blocking method
        # self._picker_robot.go_to_joint_angles(desired_js)
        print('plate moved')


    def _astor_grip(self):
        """
        STEP 4: Astor joins by gripping the other side of the plate
        """
        position = sm.SE3(0.51, 0.15, 0.87) @ sm.SE3.RPY(-90.0,0.6,90.0, unit = 'deg', order='xyz')

        plate_pose = sm.SE3(0,-0.11 - 0.2,0) @ position @ sm.SE3.Ry(180, unit='deg')

        # non-blocking method
        command = {
            "name": "GO_TO_CARTESIAN_POSE",
            "args": [plate_pose]
        }
        self._bender_robot.send_command(command)

        # # blocking method
        # self._bender_robot.go_to_cartesian_pose(plate_pose)
        print('astor gripped')

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

    def _drop_obejct(self):
        """
        STEP 5: Drop the object
        After this step, consider mission done, can send completion signal, move back home 
        or move on to another mission
        """
        self._tilt_plate()
        self._bend_plate()

    def _unbend_plate(self):
        """
        Coordinate 2 arms to unbend the plate
        """
        pass

    def _return_plate(self):
        """
        Return the plate to the picker
        """
        print('plate returned')
        pass

    def _hold_plate(self):
        """
        Hold the plate in place
        """
        print('plate held')
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
        self.mission_state = 'ENABLED'

    def disable_system(self):
        """
        Disable both robot arms
        """
        self._bender_robot.send_command('DISABLE')
        self._picker_robot.send_command('DISABLE')
        self.mission_state = 'IDLE'
        

    def stop_system(self):
        """
        Stop both robot arms
        """
        self._bender_robot.update_estop_state()
        self._picker_robot.update_estop_state()
        self.mission_state = self._picker_robot.system_state()
        

    def update_collision_object(self, side, center):

        viz_object = self._picker_robot.update_collision_object(side, center)
        self._bender_robot.update_collision_object(side, center)

        return viz_object
    
    def run(self):

        # ### different method:
        # while self.index < len(self._action_list):

        #     # check if system is activated

        #     if self._picker_robot.system_activated() is False or self._bender_robot.system_activated() is False:
        #         print('system not activated')
        #         break    
            
        #     self._action_list[self.index]()
        #     if self._picker_robot.action_is_done() is True and self._bender_robot.action_is_done() is True:
        #         self.index += 1
        #         print(f'step {self.index} executing')
        #     else:
        #         print('mission step is broken')
                
        #     time.sleep(0.5)

        # Non-blocking method:
        ### need a flow control here.
        while self.index < len(self._action_list):

            # check if system is activated

            if self._picker_robot.system_activated() is False or self._bender_robot.system_activated() is False:
                print('system not activated')
                break    


            # wait for the action to finish
        
            if self._picker_robot.get_busy_status() is False and self._bender_robot.get_busy_status() is False:

                # execute the action index and increment

                self._action_list[self.index]()
                print(f'step {self.index} executing')

            if self._picker_robot.action_is_done() is True and self._bender_robot.action_is_done() is True:
                self.index += 1

            else:
                print('mission step in progress')
                
            time.sleep(1)


    def reset_mission(self):
        """
        Reset mission to step 0"""
        self.index = 0
    

    def mission_completed(self):
        """
        Check if mission is completed
        """
        return self.index == len(self._action_list)


        
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
