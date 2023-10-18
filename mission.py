from robot import Astorino
from robot import Sawyer

import os
import copy
import time

import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
import spatialmath.base as smb
import spatialgeometry as geometry
from swift import Swift
from controller_interface import ControllerInterface



class Mission():
    def __init__(self, env : Swift, controller1: ControllerInterface, controller2: ControllerInterface):
        

        self._env = env

        # -----------------
        self.sawyer_controller = controller1
        self.astorino_controller = controller2
        #----------------
        
    def home_system(self):
        """
        Homing both robot arms"""

        self.astorino_controller.send_command('HOME')
        self.sawyer_controller.send_command('HOME')
        print(self.astorino_controller._impl._robot.q)
        
    def launch_system(self):
        """
        Launching both robot arms"""

        self.astorino_controller.launch()
        self.sawyer_controller.launch()

    def enable_system(self):
        """
        Enable both robot arms"""

        self.astorino_controller.send_command('ENABLE')
        self.sawyer_controller.send_command('ENABLE')

    def run(self):
        print('bruh')
        qgoal_a = [0, 0, 0, 0, 0, 0]
        self.astorino_controller.go_to_joint_angles(qgoal_a)
        
        # qtraj_a = rtb.jtraj(self.astorino_robot._robot.q, qgoal_a, 50).q


        # for q in qtraj_a:
        #     self.astorino_robot._robot.q = q
        #     self.env.step(0.02)
        
    def update_collision_object(self, side, center):

        viz_object = self.sawyer_controller.update_collision_object(side, center)
        self.astorino_controller.update_collision_object(side, center)

        return viz_object

    
    def test(self):
        pass

        
if __name__ == "__main__":

    env = Swift()
    env.launch(realtime= True)
    
    sawyer_robot = Sawyer(env= env)
    sawyer_controller = ControllerInterface(sawyer_robot, env= env)
    
    astorino_robot = Astorino(env= env, base= sm.SE3(0,1,0))
    astorino_controller = ControllerInterface(astorino_robot, env= env)
    
    mission = Mission(env, sawyer_controller, astorino_controller)
    mission.launch_system() # only called once when both controller havent been launched, otherwise, dont do this
    mission.enable_system() # only called when controllers need to be enabled
    mission.home_system()
