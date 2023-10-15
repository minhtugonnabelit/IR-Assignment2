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
    def __init__(self, env : Swift, sawyer_controller: ControllerInterface, astorino_controller: ControllerInterface):
        
        self.env = env

        self.sawyer_robot = sawyer_controller
        self.astorino_robot = astorino_controller
        

        
    def run(self):
        print('bruh')
        qgoal_a = [0, 0, 0, 0, 0, 0]
        # qtraj_a = rtb.jtraj(self.astorino_robot._robot.q, qgoal_a, 50).q
        self.astorino_robot.go_to_joint_angles(qgoal_a)
        
        
        # for q in qtraj_a:
        #     self.astorino_robot._robot.q = q
        #     self.env.step(0.02)
        
    
    
    def test(self):
        pass

        
if __name__ == "__main__":
    env = Swift()
    env.launch(realtime= True)
    
    sawyer_robot = Sawyer(env= env)
    sawyer_robot.add_to_env(env)
    
    astorino_robot = Astorino(env= env)
    astorino_robot.add_to_env(env) 
    
    
    
    # Mission(env= env, sawyer_controller= sawyer_robot, astorino_controller= astorino_robot).run
    
    Mission.run(self= astorino_robot)
