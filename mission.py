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
        
        # init swift environment
        self.env = env

        # init robots controller inside mission
        self.sawyer_robot = controller1
        self.astorino_robot = controller2
                
    def run(self):
        print('bruh')
        qgoal_a = [0, 0, 0, 0, 0, 0]
        self.astorino_robot.go_to_joint_angles(qgoal_a)
        # qtraj_a = rtb.jtraj(self.astorino_robot._robot.q, qgoal_a, 50).q

        # for q in qtraj_a:
        #     self.astorino_robot._robot.q = q
        #     self.env.step(0.02)
        
    def update_collision_object(self, object):

        self.sawyer_robot.update_collision_object(object)
        self.astorino_robot.update_collision_object(object)

    
    def test(self):
        pass

        
if __name__ == "__main__":

    env = Swift()
    env.launch(realtime= True)
    
    sawyer_robot = Sawyer(env= env)
    sawyer_robot.add_to_env(env)
    
    astorino_robot = Astorino(env= env)
    astorino_robot.add_to_env(env) 
    
    Mission(env= env, sawyer_controller= sawyer_robot, astorino_controller= astorino_robot).run()
