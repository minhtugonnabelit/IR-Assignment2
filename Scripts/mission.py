import numpy as np
import spatialmath.base as smb
import spatialmath as sm
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
from swift import Swift


ur = rtb.models.DH.UR3()
ur.q = np.zeros(6)
ur.teach(ur.q)

def get_body_ellipsoid(robot: rtb.DHRobot):
    
    



    pass