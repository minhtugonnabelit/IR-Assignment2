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

    STEP = 50

    def __init__(self, plates_list, workcell : WorkCell, picker_robot : ControllerInterface, bender_robot : ControllerInterface):
        

        # -----------------
        # Use getter function (TODO) of these controllers to keep track of their current position /
        self._workcell = workcell
        self._picker_robot = picker_robot
        self._bender_robot = bender_robot

        self._plates_list = plates_list
        self.INITPOSES = []
        self.LIFTPOSES = []
        self.HANGPOSES = []

        for plate in self._plates_list:
            self.INITPOSES.append(plate.get_pose())
            self.LIFTPOSES.append(sm.SE3(0,0,0.03) @ plate.get_pose())
            self.HANGPOSES.append(sm.SE3(0,0,0.03) @ sm.SE3(0.1,0,0) @ plate.get_pose())

        self.JOINEDPOSE = sm.SE3(0.5, 0, 0.9) @ sm.SE3.RPY(90.0,0.0,0.0, unit = 'deg', order='xyz')
        self.TILTEDPOSE = self.JOINEDPOSE @ sm.SE3.Rx(np.pi/3)


        # end effector pose relative to plate base
        self._PICKER_GRIP_POSE = sm.SE3(0.25,0,0) @ sm.SE3.RPY(0,-90,-180, unit = 'deg', order='xyz')
        self._BENDER_GRIP_POSE = sm.SE3(-0.23,0,0) @ sm.SE3.RPY(-90,90,-90, unit = 'deg', order='xyz') 

        #----------------
        self._cell_center = self._workcell.get_cell_center()
        self._cart_location = self._workcell.get_cart_location()


        # index to track current step
        self.index = 0
        self.plate_index = 0
        self.done = True
        self.mission_state = 'IDLE'


        self._action_list = [
            self._grip_plate_edge,
            self._lift_plate,
            self._hang_plate,
            self._move_plate,
            self._astor_grip,
            self._tilt_plate,
            self._bend_plate,
            self._drop_obejct,
            self._unbend_plate,
            self._astor_release,
            self._hold_plate,
            self._return_plate,
        ]

        self._action_list2 = []
        

    def _home_system(self):
        """
        STEP 1: Homing both robot arms, ready to pick
        """
        # non-blocking method
        self._bender_robot.send_command('HOME')
        # self._picker_robot.send_command('HOME')
        self._picker_robot.go_to_home()
        self._bender_robot.open_gripper()
        print('system homed')
    

    def _grip_plate_edge(self, plt_index):
        """
        STEP 2 : Grip the edge of the plate
        After this step, use the _plate_position to coordinate the arm(s)
        """

        ## Approach the plate and grip 
        """
        Supposed to get plate location and orientation from vision system
        OR directly get plate pose from object and use it to define grasp pose
        
        Both method currently not available, so use a fixed pose for now"""

        plate_pose = self._plates_list[plt_index].get_pose()

        grip_pose = plate_pose @ self._PICKER_GRIP_POSE
        current_pose = self._picker_robot.get_ee_pose()    

        pose_ori = grip_pose.A[0:3,0:3]
        path_sawyer = np.empty((self.STEP, 3))

        for i in range(self.STEP):
            t = i / (self.STEP - 1) # Interpolation parameter [0, 1]
            path_sawyer[i, :] = (1 - t)*current_pose.A[0:3,3] +t*grip_pose.A[0:3,3]

        index = 0
        while index < len(path_sawyer) and self._picker_robot.system_activated() is True:

            pose = sm.SE3(path_sawyer[index])
            pose.A[0:3,0:3] = pose_ori          
            self._picker_robot.single_step_cartesian(pose, 0.01)

            index += 1
            self.done = self._picker_robot.is_arrived(grip_pose)
            time.sleep(0.01)

        # # grip_pose = sm.SE3(-0.09,0.91,0.91) 
        # # non-blocking method
        # command = {
        #     "name": "GO_TO_CARTESIAN_POSE",
        #     "args": [grip_pose]s
        # }
        # self._picker_robot.send_command(command)

        # # blocking method
        # self._picker_robot.go_to_cartesian_pose(grip_pose)

        print('plate gripped')


    def _lift_plate(self, plt_index):
        """
        Lift the plate to a safe height
        """

        # close gripper
        self._picker_robot.close_gripper()

        # lift the plate
        grip_pose = self.LIFTPOSES[plt_index] @ self._PICKER_GRIP_POSE
        current_pose = self._picker_robot.get_ee_pose()    

        pose_ori = grip_pose.A[0:3,0:3]
        path_sawyer = np.empty((self.STEP, 3))

        for i in range(self.STEP):
            t = i / (self.STEP - 1) # Interpolation parameter [0, 1]
            path_sawyer[i, :] = (1 - t)*current_pose.A[0:3,3] +t*grip_pose.A[0:3,3]

        index = 0
        while index < len(path_sawyer) and self._picker_robot.system_activated():

            pose = sm.SE3(path_sawyer[index])
            pose.A[0:3,0:3] = pose_ori          
            self._picker_robot.single_step_cartesian(pose, 0.01)

            plate_pose = self._picker_robot.get_ee_pose().A @ np.linalg.inv(self._PICKER_GRIP_POSE)
            self._plates_list[plt_index].move_flat_plate(sm.SE3(plate_pose))
            index += 1
            self.done = self._picker_robot.is_arrived(grip_pose)
            time.sleep(0.01)

        # # non-blocking method
        # command = {
        #     "name": "GO_TO_CARTESIAN_POSE",
        #     "args": [lift_pose]
        # }
        # self._picker_robot.send_command(command)

        # for i in range(50):
        #     self._plate.move_flat_plate(lift_pose @ sm.SE3(0,0,0.1/50))
        #     time.sleep(0.01)

        # # blocking method
        # self._picker_robot.go_to_cartesian_pose(lift_pose)

        print('plate lifted')


    def _hang_plate(self, plt_index):
        """
        Hang the plate to the printer
        """

        # hang the plate
        hang_pose = self.HANGPOSES[plt_index] @ self._PICKER_GRIP_POSE
        current_pose = self._picker_robot.get_ee_pose()

        pose_ori = hang_pose.A[0:3,0:3]
        path_sawyer = np.empty((self.STEP, 3))

        for i in range(self.STEP):
            t = i / (self.STEP - 1) # Interpolation parameter [0, 1]
            path_sawyer[i, :] = (1 - t)*current_pose.A[0:3,3] +t*hang_pose.A[0:3,3]

        index = 0
        while index < len(path_sawyer) and self._picker_robot.system_activated():

            pose = sm.SE3(path_sawyer[index])
            pose.A[0:3,0:3] = pose_ori          
            self._picker_robot.single_step_cartesian(pose, 0.01)

            plate_pose = self._picker_robot.get_ee_pose().A @ np.linalg.inv(self._PICKER_GRIP_POSE)
            self._plates_list[plt_index].move_flat_plate(sm.SE3(plate_pose))
            index += 1
            self.done = self._picker_robot.is_arrived(hang_pose)
            time.sleep(0.01)

        # non-blocking method
        # command = {
        #     "name": "GO_TO_CARTESIAN_POSE",
        #     "args": [hang_pose]
        # }
        # self._picker_robot.send_command(command)

        # # blocking method
        # self._picker_robot.go_to_cartesian_pose(hang_pose)

        print('plate hanged')


    def _move_plate(self, plt_index):
        """
        ### STEP 3: 
        Move the print plate to the specified position
       
        manually generate a path other than a straight line to avoid singularity 
        OR use the feedback control to auto complete the path"""

        # move the plate
        sawyer_gripper_pose = self.JOINEDPOSE @ self._PICKER_GRIP_POSE
        desired_js = self._picker_robot.get_robot().ikine_LM(sawyer_gripper_pose, q0 = self._picker_robot.get_joint_angles()).q
        path_sawyer = rtb.jtraj(self._picker_robot.get_joint_angles(), desired_js, self.STEP).q

        index = 0
        while index < len(path_sawyer) and self._picker_robot.system_activated():

            self._picker_robot.single_step_joint(path_sawyer[index], 0.01)

            plate_pose = self._picker_robot.get_ee_pose().A @ np.linalg.inv(self._PICKER_GRIP_POSE)
            self._plates_list[plt_index].move_flat_plate(sm.SE3(plate_pose))
            index += 1
            self.done = self._picker_robot.is_arrived(sawyer_gripper_pose)
            time.sleep(0.01)

        # # non-blocking method
        # command = {
        #     "name": "GO_TO_JOINT_ANGLES",
        #     "args": [desired_js]
        # }
        # self._picker_robot.send_command(command)

        # # blocking method
        # self._picker_robot.go_to_joint_angles(desired_js)

        print('plate moved')


    def _astor_grip(self, plt_index):
        """
        STEP 4: Astor joins by gripping the other side of the plate
        """

        # plate_desired_pose = sm.SE3(0.5, 0, 0.87) @ sm.SE3.RPY(90.0, 0.0, 0.0, unit = 'deg', order='xyz')
        astorino_gripper_pose = self.JOINEDPOSE @ self._BENDER_GRIP_POSE
        path_astor = rtb.ctraj(self._bender_robot.get_ee_pose(), astorino_gripper_pose, self.STEP)
    
        index = 0
        while index < len(path_astor) and self._bender_robot.system_activated():
            self._bender_robot.single_step_cartesian(path_astor[index], 0.01)
            index += 1
            self.done = self._bender_robot.is_arrived(astorino_gripper_pose)
            time.sleep(0.01)

        self._bender_robot.close_gripper()

        # non-blocking method
        # command = {
        #     "name": "GO_TO_CARTESIAN_POSE",
        #     "args": [plate_pose]
        # }
        # self._bender_robot.send_command(command)

        # # blocking method
        # self._bender_robot.go_to_cartesian_pose(plate_pose)

        print('astor gripped')

    def _tilt_plate(self, plt_index):
        """
        Coordinate 2 arms to tilt the plate to a specified orientation to drop the object
        - @param forth: True for tilting forth, False for back
        """

        path_plate = rtb.ctraj(self._plates_list[plt_index].get_pose(), self.TILTEDPOSE, 50)
        picker_last_pose = path_plate[-1] @ self._PICKER_GRIP_POSE
        bender_last_pose = path_plate[-1] @ self._BENDER_GRIP_POSE

        for pose in path_plate:

            picker_grip_pose = pose @ self._PICKER_GRIP_POSE
            bend_grip_pose = pose @ self._BENDER_GRIP_POSE
            
            self._picker_robot.single_step_cartesian(picker_grip_pose, 0.01)
            self._bender_robot.single_step_cartesian(bend_grip_pose, 0.01)

            self._plates_list[plt_index].move_flat_plate(pose)
            
            self.done = self._picker_robot.is_arrived(picker_last_pose) and self._bender_robot.is_arrived(bender_last_pose)

            time.sleep(0.01)


    def _bend_plate(self, plt_index):
        """
        Coordinate 2 arms to bend the plate and return to unbend position
        """
        step = 20
        self.all_seg = []

        for i in range(step):

            seg_array = []
            _pick, _bend = self._plates_list[plt_index].bend(i, seg_array)

            # somehow this copy is real necessary, otherwise the pose will be updated
            pick = copy.deepcopy(_pick)
            bend = copy.deepcopy(_bend)

            # assign relative orientation of the gripper in plate center frame to the extracted edges pose 
            pick_ori = pick.A[0:3,0:3] @ self._PICKER_GRIP_POSE.A[0:3,0:3]
            pick.A[0:3,0:3] = pick_ori
            bend_ori = bend.A[0:3,0:3] @ self._BENDER_GRIP_POSE.A[0:3,0:3]
            bend.A[0:3,0:3] = bend_ori

            # position of the grasping pose is kept
            picker_grip_pose = pick @ sm.SE3(np.linalg.inv(self._picker_robot.get_robot().gripper_offset.A))
            bender_grip_pose = bend @ sm.SE3(np.linalg.inv(self._bender_robot.get_robot().gripper_offset.A))

            # send motion command
            self._picker_robot.single_step_cartesian(picker_grip_pose, 0.01)
            self._bender_robot.single_step_cartesian(bender_grip_pose, 0.01)

            self.all_seg.append(seg_array)
            time.sleep(0.01)
        

    def _drop_obejct(self, plt_index):
        """
        STEP 5: Drop the object
        After this step, consider mission done, can send completion signal, move back home 
        or move on to another mission
        """
        pass

    def _unbend_plate(self, plt_index):
        """
        Coordinate 2 arms to unbend the plate
        """

        for i, seg_array in enumerate(reversed(self.all_seg)):

            _pick, _bend = self._plates_list[plt_index].unbend(seg_array)

            # as noted above
            pick = copy.deepcopy(_pick)
            bend = copy.deepcopy(_bend)

            # assign relative orientation of the gripper in plate center frame to the extracted edges pose 
            pick_ori = pick.A[0:3,0:3] @ self._PICKER_GRIP_POSE.A[0:3,0:3]
            pick.A[0:3,0:3] = pick_ori
            bend_ori = bend.A[0:3,0:3] @ self._BENDER_GRIP_POSE.A[0:3,0:3]
            bend.A[0:3,0:3] = bend_ori

            # position of the grasping pose is kept
            picker_grip_pose = pick @ sm.SE3(np.linalg.inv(self._picker_robot.get_robot().gripper_offset.A))
            bender_grip_pose = bend @ sm.SE3(np.linalg.inv(self._bender_robot.get_robot().gripper_offset.A))

            # send motion command
            self._picker_robot.single_step_cartesian(picker_grip_pose, 0.01)
            self._bender_robot.single_step_cartesian(bender_grip_pose, 0.01)

            time.sleep(0.01)

    
    def _astor_release(self, i):
        """
        STEP 6: Astor release the plate
        """
        self._bender_robot.open_gripper()
        # path_astor = rtb.ctraj(self._bender_robot.get_ee_pose(), self._bender_robot.get_ee_pose() @ sm.SE3(0,0,-0.1), self.STEP)

        self._bender_robot.go_to_cartesian_pose(self._bender_robot.get_ee_pose() @ sm.SE3(0,0,-0.1))
    

    def _hold_plate(self, plt_index):
        """
        Hold the plate in place
        Need a way to combine traj
        """
        hang_pose = self.HANGPOSES[plt_index] @ self._PICKER_GRIP_POSE
        fix_pose = sm.SE3(-0.15,0,0.1) @ self._picker_robot.get_ee_pose() 
        fix_path = rtb.ctraj(self._picker_robot.get_ee_pose(), fix_pose, self.STEP)
        

        index = 0
        while index < len(fix_path) and self._picker_robot.system_activated():
            self._picker_robot.single_step_cartesian(fix_path[index], 0.01)
            plate_pose = self._picker_robot.get_ee_pose().A @ np.linalg.inv(self._PICKER_GRIP_POSE)
            self._plates_list[plt_index].move_flat_plate(sm.SE3(plate_pose))
            index += 1
            self.done = self._picker_robot.is_arrived(hang_pose)
            time.sleep(0.01)

        path_sawyer = rtb.ctraj(fix_pose, hang_pose, self.STEP)

        index = 0
        while index < len(path_sawyer) and self._picker_robot.system_activated():
            self._picker_robot.single_step_cartesian(path_sawyer[index], 0.01)
            plate_pose = self._picker_robot.get_ee_pose().A @ np.linalg.inv(self._PICKER_GRIP_POSE)
            self._plates_list[plt_index].move_flat_plate(sm.SE3(plate_pose))
            index += 1
            self.done = self._picker_robot.is_arrived(hang_pose)
            time.sleep(0.01)
        

        print('plate held')
        pass

    def _return_plate(self, plt_index):
        """
        Return the plate to the printer
        """
        return_pose = self.INITPOSES[plt_index] @ self._PICKER_GRIP_POSE
        current_pose = self._picker_robot.get_ee_pose()

        pose_ori = return_pose.A[0:3,0:3]
        path_sawyer = np.empty((self.STEP, 3))

        for i in range(self.STEP):
            t = i / (self.STEP - 1) # Interpolation parameter [0, 1]
            path_sawyer[i, :] = (1 - t)*current_pose.A[0:3,3] +t*return_pose.A[0:3,3]

        index = 0
        while index < len(path_sawyer) and self._picker_robot.system_activated():
            pose = sm.SE3(path_sawyer[index])
            pose.A[0:3,0:3] = pose_ori
            self._picker_robot.single_step_cartesian(pose, 0.01)
            plate_pose = self._picker_robot.get_ee_pose().A @ np.linalg.inv(self._PICKER_GRIP_POSE)
            self._plates_list[plt_index].move_flat_plate(sm.SE3(plate_pose))

            index += 1
            self.done = self._picker_robot.is_arrived(return_pose)
            time.sleep(0.01)

        self._picker_robot.open_gripper()  

        if plt_index == len(self._plates_list) - 1:
            self._home_system()      

        print('plate returned')

    # LLT ==-----------------
    def system_state(self):
        return self.mission_state


    # TAM ----------------------------------------------------------------------------
    def step_is_done(self):
        return self.done
    

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

        while self.plate_index < len(self._plates_list):

            if self._picker_robot.system_activated() is False or self._bender_robot.system_activated() is False:
                print('system not activated')
                break 
            
            if self.plate_index == 0 and self.index == 0:
                self._home_system()
            
            while self.index < len(self._action_list):

                # check if system is activated
                if self._picker_robot.system_activated() is False or self._bender_robot.system_activated() is False:
                    print('system not activated')
                    break   

                print(f'Step {self.index} executing')
                self._action_list[self.index](self.plate_index)
                
                if self.step_is_done():
                    self.index += 1

                time.sleep(0.5)

            if self.task_completed():   

                print('task completed') 
                self.plate_index += 1
                

                if  self.plate_index <= len(self._plates_list)-1:

                    # reset task index
                    print('Task index reset!')
                    self.index = 0

        

    def reset_mission(self):
        """
        Reset mission to step 0"""
        self.index = 0
        self.plate_index = 0
    

    def task_completed(self):
        """
        Check if task is completed
        """
        return self.index == len(self._action_list) and self.step_is_done()
    

    def mission_completed(self):
        """
        Check if mission is completed
        """
        
        return self.index == len(self._action_list) and self.plate_index == len(self._plates_list) and self.step_is_done()



        
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
