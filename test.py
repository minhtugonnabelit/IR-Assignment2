import roboticstoolbox as rtb
import numpy as np
from swift import Swift
from controller import Controller

env = Swift()
env.launch(realtime=True)
ur = rtb.models.UR5()
ur.q = [0, -np.pi/2, np.pi/3, -np.pi/2, 0, 0]
env.add(ur)

desired = ur.fkine([np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, 0, 0])

controller = Controller(ur, env)
controller.go_to_CartesianPose(desired, time=5)

env.hold()