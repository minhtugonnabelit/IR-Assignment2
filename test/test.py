import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
import spatialgeometry as geometry
import spatialmath.base as smb
from swift import Swift
from controller import Controller
from robot import Sawyer



env = Swift()


sawyer = Sawyer(env)
sawyer_controller = Controller(sawyer, env)

object = geometry.Cuboid([0.1, 0.1, 0.1], pose=sm.SE3(sawyer.base.A @ smb.transl(0.5,0.2,0.2)), name='object')
# sawyer_controller.add_object(object)
