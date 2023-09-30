import roboticstoolbox as rtb
import numpy as np

from robot.m_DHRobot3D import M_DHRobot3D

class Jaka(M_DHRobot3D):
    
    def __init__(self, links, link3D_names, link3d_dir, name=None, qtest=None, qtest_transforms=None):
        super().__init__(links, link3D_names, link3d_dir, name, qtest, qtest_transforms)