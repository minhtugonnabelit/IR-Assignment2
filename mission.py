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

    STEP = 100

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
        self._PICKER_GRIP_POSE = sm.SE3(0.1,0,0) @ sm.SE3.RPY(0,-90,-180, unit = 'deg', order='xyz')
        self._BENDER_GRIP_POSE = sm.SE3(-0.11,0,0) @ sm.SE3.RPY(-90,90,-90, unit = 'deg', order='xyz') 

        #----------------
        self._cell_center = self._workcell.get_cell_center()
        self._cart_location = self._workcell.get_cart_location()


        # index to track current step
        self.index = 0
        self.plate_index = 0
        self.done = True
        self.mission_state = 'IDLE'
        
        
        # Action list for the mission
        self._action_list = [
            self._grip_plate_edge,
            self._lift_plate,
            self._hang_plate,
            self._move_plate,
            self._astor_grip,
            self._tilt_plate,
            self._bend_plate,
            self._unbend_plate,
            self._astor_release,
            self._hold_plate,
            self._return_plate,
        ]

        self._action_list2 = []
        
    def _collision_avoidance(self, robot : ControllerInterface, plate : Plate,  bring_plate: bool,  bring_part: bool, gripper_pose, desired_js, right_to_left: bool):

        # Defining avoidance direction
        if right_to_left: 
            move_through_obstacle = 0.011
        else: move_through_obstacle = -0.011 # move left to right

        # Start avoiding obstacle
        while robot.robot_is_collided():

            # go up if got plate gripped, else move backward to avoid
            if bring_plate:
                fix_pose = sm.SE3(0,0,0.005) @ robot.get_ee_pose()
            else: fix_pose = robot.get_ee_pose() @ sm.SE3(0,0,-0.07)
            
            robot.single_step_cartesian(fix_pose, 0.02)
            
            # option to animate the plate or not
            if bring_plate:
                plate_pose = robot.get_ee_pose().A @ np.linalg.inv(self._PICKER_GRIP_POSE)
                plate.move_flat_plate(sm.SE3(plate_pose), bring_part)   
            
        # minor fixing to completely avoid
        for _ in range(20):
            if bring_plate:
                fix_pose_2 = robot.get_ee_pose() @ sm.SE3(-0.001, move_through_obstacle, 0)
            else: 
                fix_pose_2 = robot.get_ee_pose() @ sm.SE3(0, move_through_obstacle, 0)
                
            robot.single_step_cartesian(fix_pose_2, 0.02)
            if bring_plate:
                plate_pose = robot.get_ee_pose().A @ np.linalg.inv(self._PICKER_GRIP_POSE)
                plate.move_flat_plate(sm.SE3(plate_pose), bring_part)  
                
        # try to get back to current
        new_path = rtb.jtraj(robot.get_joint_angles(), desired_js, self.STEP).q
        index = 0
        while index < len(new_path) and robot.system_activated():

            robot.single_step_joint(new_path[index], 0.01)
            if bring_plate:
                plate_pose = robot.get_ee_pose().A @ np.linalg.inv(self._PICKER_GRIP_POSE)
                plate.move_flat_plate(sm.SE3(plate_pose), bring_part)
            index += 1
            self.done = robot.is_arrived(gripper_pose)

    def _home_system(self):
        """
        STEP 1: Homing both robot arms, ready to pick
        """
        # non-blocking method
        self._bender_robot.send_command('HOME')        
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
        s = rtb.trapezoidal(0,1,self.STEP).s
        for i in range(self.STEP):
            path_sawyer[i, :] = (1 - s[i])*current_pose.A[0:3,3] + s[i]*grip_pose.A[0:3,3]

        path = []
        for pose in path_sawyer:
            p = sm.SE3(pose)
            p.A[:3,:3] = pose_ori
            path.append(p)

        index = 0
        while index < len(path_sawyer) and self._picker_robot.system_activated() is True and not self._picker_robot.robot_is_collided():
            self._picker_robot.single_step_cartesian(path[index], 0.02)
            index += 1
            self.done = self._picker_robot.is_arrived(grip_pose)

        desired_js = self._picker_robot.get_robot().ikine_LM(grip_pose, q0 = self._picker_robot.get_joint_angles()).q
        if self._picker_robot.robot_is_collided():
            self._collision_avoidance(robot= self._picker_robot, 
                                      plate= self._plates_list[plt_index], 
                                      bring_plate= False,
                                      bring_part= False,
                                      gripper_pose= grip_pose,
                                      desired_js= desired_js,
                                      right_to_left= True)

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
        path_sawyer = np.empty((30, 3))

        for i in range(30):
            t = i / (30 - 1) # Interpolation parameter [0, 1]
            path_sawyer[i, :] = (1 - t)*current_pose.A[0:3,3] +t*grip_pose.A[0:3,3]

        path = []
        for pose in path_sawyer:
            p = sm.SE3(pose)
            p.A[:3,:3] = pose_ori
            path.append(p)

        index = 0
        while index < len(path_sawyer) and self._picker_robot.system_activated():

            self._picker_robot.single_step_cartesian(path[index], 0.02)

            plate_pose = self._picker_robot.get_ee_pose().A @ np.linalg.inv(self._PICKER_GRIP_POSE)
            self._plates_list[plt_index].move_flat_plate(sm.SE3(plate_pose))
            index += 1
            self.done = self._picker_robot.is_arrived(grip_pose)

        print('plate lifted')


    def _hang_plate(self, plt_index):
        """
        Hang the plate to the printer
        """

        # hang the plate
        hang_pose = self.HANGPOSES[plt_index] @ self._PICKER_GRIP_POSE
        current_pose = self._picker_robot.get_ee_pose()
        
        # hangstep = self.STEP/2

        pose_ori = hang_pose.A[0:3,0:3]
        path_sawyer = np.empty((self.STEP, 3))
        s = rtb.trapezoidal(0,1,self.STEP).s
        for i in range(self.STEP):
            path_sawyer[i, :] = (1 - s[i])*current_pose.A[0:3,3] +s[i]*hang_pose.A[0:3,3]


        index = 0
        while index < len(path_sawyer) and self._picker_robot.system_activated():

            pose = sm.SE3(path_sawyer[index])
            pose.A[0:3,0:3] = pose_ori          
            self._picker_robot.single_step_cartesian(pose, 0.02)

            plate_pose = self._picker_robot.get_ee_pose().A @ np.linalg.inv(self._PICKER_GRIP_POSE)
            self._plates_list[plt_index].move_flat_plate(sm.SE3(plate_pose))
            index += 1
            self.done = self._picker_robot.is_arrived(hang_pose)


        print('plate hanged')

    
    def _move_plate(self, plt_index): # need avoidance
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
        while index < len(path_sawyer) and self._picker_robot.system_activated() and not self._picker_robot.robot_is_collided():

            self._picker_robot.single_step_joint(path_sawyer[index], 0.02)

            plate_pose = self._picker_robot.get_ee_pose().A @ np.linalg.inv(self._PICKER_GRIP_POSE)
            self._plates_list[plt_index].move_flat_plate(sm.SE3(plate_pose))
            index += 1
            self.done = self._picker_robot.is_arrived(sawyer_gripper_pose)
            
        #------------ collision
        if self._picker_robot.robot_is_collided():
            self._collision_avoidance(robot= self._picker_robot, 
                                      plate= self._plates_list[plt_index], 
                                      bring_plate= True,
                                      bring_part= True,
                                      gripper_pose= sawyer_gripper_pose,
                                      desired_js= desired_js,
                                      right_to_left= True)
              
        #--------- end collision

        print('plate moved')


    def _astor_grip(self, plt_index):
        """
        STEP 4: Astor joins by gripping the other side of the plate
        """
        if plt_index > 0:
            self._BENDER_GRIP_POSE = sm.SE3.Rx(-np.pi/90) @ self._BENDER_GRIP_POSE
        astorino_gripper_pose = self.JOINEDPOSE @ self._BENDER_GRIP_POSE
        path_astor = rtb.ctraj(self._bender_robot.get_ee_pose(), astorino_gripper_pose, self.STEP)
    
        index = 0
        while index < len(path_astor) and self._bender_robot.system_activated():
            self._bender_robot.single_step_cartesian(path_astor[index], 0.02)
            index += 1
            self.done = self._bender_robot.is_arrived(astorino_gripper_pose)

        self._bender_robot.close_gripper()


        print('astor gripped')

    def _tilt_plate(self, plt_index):
        """
        Coordinate 2 arms to tilt the plate to a specified orientation to drop the object
        - @param forth: True for tilting forth, False for back
        """

        path_plate = rtb.ctraj(self._plates_list[plt_index].get_pose(), self.TILTEDPOSE, 30)
        picker_last_pose = path_plate[-1] @ self._PICKER_GRIP_POSE
        bender_last_pose = path_plate[-1] @ self._BENDER_GRIP_POSE

        for pose in path_plate:

            picker_grip_pose = pose @ self._PICKER_GRIP_POSE
            bend_grip_pose = pose @ self._BENDER_GRIP_POSE
            
            self._picker_robot.single_step_cartesian(picker_grip_pose, 0.02)
            self._bender_robot.single_step_cartesian(bend_grip_pose, 0.02)

            self._plates_list[plt_index].move_flat_plate(pose)
            
            self.done = self._picker_robot.is_arrived(picker_last_pose) and self._bender_robot.is_arrived(bender_last_pose)

    def _bend_plate(self, plt_index):
        """
        Coordinate 2 arms to bend the plate and return to unbend position
        """
        step = 50
        self.all_seg = []
        step_bunny_slide = 0.2
        step_bunny = step_bunny_slide / step

        s = rtb.trapezoidal(0, np.deg2rad(12), step).s


        for i in range(step):

            seg_array = []
            _pick, _bend = self._plates_list[plt_index].bend(s[i], seg_array)

            # somehow this copy is real necessary, otherwise the pose will be updated
            pick = copy.deepcopy(_pick)
            bend = copy.deepcopy(_bend)

            # assign relative orientation of the gripper in plate center frame to the extracted edges pose 
            pick_ori = pick.A[0:3,0:3] @ self._PICKER_GRIP_POSE.A[0:3,0:3]
            pick.A[0:3,0:3] = pick_ori
            bend_ori = bend.A[0:3,0:3] @ self._BENDER_GRIP_POSE.A[0:3,0:3]
            bend.A[0:3,0:3] = bend_ori

            # drop bunny
            self._plates_list[plt_index].drop_bunny_step(step= step_bunny*i)

            # position of the grasping pose is kept
            picker_grip_pose = pick
            bender_grip_pose = bend 

            
            # send motion command
            self._picker_robot.single_step_cartesian(picker_grip_pose, 0.01)
            self._bender_robot.single_step_cartesian(bender_grip_pose, 0.01)

            self.all_seg.append(seg_array)
                    
    def _unbend_plate(self, plt_index):
        """
        Coordinate 2 arms to unbend the plate
        """
        step = len(self.all_seg)
        step_bunny_drop = 0.02
        step_bunny = step_bunny_drop / step
        
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

            # drop bunny
            self._plates_list[plt_index].drop_bunny_step_box(step= step_bunny*i)

            # position of the grasping pose is kept
            picker_grip_pose = pick
            bender_grip_pose = bend

            # send motion command
            self._picker_robot.single_step_cartesian(picker_grip_pose, 0.01)
            self._bender_robot.single_step_cartesian(bender_grip_pose, 0.01)

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
        fix_path = rtb.ctraj(self._picker_robot.get_ee_pose(), fix_pose, 30)
        

        index = 0
        while index < len(fix_path) and self._picker_robot.system_activated():
            self._picker_robot.single_step_cartesian(fix_path[index], 0.01)
            plate_pose = self._picker_robot.get_ee_pose().A @ np.linalg.inv(self._PICKER_GRIP_POSE)
            self._plates_list[plt_index].move_flat_plate(sm.SE3(plate_pose), part_relate = False)
            index += 1
            self.done = self._picker_robot.is_arrived(hang_pose)


        # Return plate -> Go through collision object
        path_sawyer = rtb.ctraj(fix_pose, hang_pose, self.STEP)

        index = 0
        while index < len(path_sawyer) and self._picker_robot.system_activated() and not self._picker_robot.robot_is_collided():
            self._picker_robot.single_step_cartesian(path_sawyer[index], 0.01)
            plate_pose = self._picker_robot.get_ee_pose().A @ np.linalg.inv(self._PICKER_GRIP_POSE)
            self._plates_list[plt_index].move_flat_plate(sm.SE3(plate_pose), part_relate = False)
            index += 1
            self.done = self._picker_robot.is_arrived(hang_pose)
        
        #------------ collision
        if self._picker_robot.robot_is_collided():
            desired_js = self._picker_robot.get_robot().ikine_LM(hang_pose, q0 = self._picker_robot.get_joint_angles()).q
            self._collision_avoidance(robot= self._picker_robot, 
                                      plate= self._plates_list[plt_index], 
                                      bring_plate= True,
                                      bring_part= False,
                                      gripper_pose= hang_pose,
                                      desired_js= desired_js,
                                      right_to_left= False)
              
        #--------- end collision
        

        print('plate held')
        pass

    def _return_plate(self, plt_index):
        """
        Return the plate to the printer
        """
        return_pose = self.INITPOSES[plt_index] @ self._PICKER_GRIP_POSE
        current_pose = self._picker_robot.get_ee_pose()

        pose_ori = return_pose.A[0:3,0:3]
        path_sawyer = np.empty((30, 3))

        for i in range(30):
            t = i / (30 - 1) # Interpolation parameter [0, 1]
            path_sawyer[i, :] = (1 - t)*current_pose.A[0:3,3] +t*return_pose.A[0:3,3]

        index = 0
        while index < len(path_sawyer) and self._picker_robot.system_activated():
            pose = sm.SE3(path_sawyer[index])
            pose.A[0:3,0:3] = pose_ori
            self._picker_robot.single_step_cartesian(pose, 0.01)
            plate_pose = self._picker_robot.get_ee_pose().A @ np.linalg.inv(self._PICKER_GRIP_POSE)
            self._plates_list[plt_index].move_flat_plate(sm.SE3(plate_pose), part_relate = False)

            index += 1
            self.done = self._picker_robot.is_arrived(return_pose)


        self._picker_robot.open_gripper()  

        if plt_index == len(self._plates_list) - 1:

            neutral_js = self._picker_robot.get_robot().neutral
            home_pose = self._picker_robot.get_ee_pose(neutral_js)
            path_sawyer = rtb.jtraj(self._picker_robot.get_joint_angles(), neutral_js, self.STEP).q
            
            index = 0
            while index < len(path_sawyer) and self._picker_robot.system_activated() and not self._picker_robot.robot_is_collided():
                self._picker_robot.single_step_joint(path_sawyer[index], 0.01)
                index += 1
                self.done = self._picker_robot.is_arrived(home_pose)

            if self._picker_robot.robot_is_collided():
                self._collision_avoidance(robot= self._picker_robot, 
                                        plate= self._plates_list[plt_index], 
                                        bring_plate= False,
                                        bring_part= False,
                                        gripper_pose= home_pose  ,
                                        desired_js= neutral_js,
                                        right_to_left= False)

            
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
        """
        Run the mission
        """

        while self.plate_index < len(self._plates_list):

            if self._picker_robot.system_activated() is False or self._bender_robot.system_activated() is False:
                print('system not activated')
                break 
            
            if self.plate_index == 0 and self.index == 0:
                self._home_system()
            
            
            # Loop to go through individual steps for each plate
            while self.index < len(self._action_list):

                # check if system is activated
                if self._picker_robot.system_activated() is False or self._bender_robot.system_activated() is False:
                    print('system not activated')
                    break   

                print(f'Step {self.index} executing')
                self._action_list[self.index](self.plate_index)
                
                if self.step_is_done():
                    self.index += 1
            

            # check if task is completed
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
