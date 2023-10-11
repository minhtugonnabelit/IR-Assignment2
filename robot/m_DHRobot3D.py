##  @file
#   @brief A 3D Robot Class defined by standard DH parameters.
#   @author Ho Minh Quang Ngo and Anh Minh Tu
#   @date Jul 20, 2023

import swift
import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as geometry
import spatialmath.base as spb
import os
import copy
from abc import ABC

# Useful variables
from math import pi
from ir_support import RectangularPrism, line_plane_intersection

# -----------------------------------------------------------------------------------#
class M_DHRobot3D(rtb.DHRobot, ABC):
    """
    This abstract class inherits from the DHRobot class of the `Robotics Toolbox in Python`\n 
    It represents a 3D Robot defined in standard DH parameters, that can be displayed in `Swift` simulator

        Parameters:
        -----------------------------------------------------------
        - `links`: list of DH links `roboticstoolbox.DHLink` to construct the robot
        - `link3D_names`: dictionary for names of the robot object in the directory, e.g. {link0: 'base.dae', link1: 'shoulder.dae'...}
        - `link3D_dir`: absolute path to the 3D files
        - `name`: name of the robot 
        - `qtest`: an input joint config as a list to calibrate the 3D model. Number of elements in `qtest` is number of robot joints
        - `qtest_transforms`: transforms of the 3D models to match the config by `qtest`.\n 
                            Number of elements in `qtest_transforms`= number of elements in `q_test` plus 1.\n
                            The first element is the transform of the global coordinate to the robot.  
    """

    def __init__(self, links, link3D_names, link3d_dir, name = None, qtest = None, qtest_transforms = None):

        super().__init__(links, name = name)
        self.link3D_names = link3D_names
        
        if qtest is None: # default qtest
            qtest = [0 for _ in range(self.n)] 
        if qtest_transforms is None: # default transforms
            qtest_transforms = [np.eye(4) for _ in range(self.n + 1)]
        
        self._link3D_dir = link3d_dir
        self._qtest = qtest
        self._qtest_transforms = qtest_transforms
        # self.neutral = np.zeros(np.size(self.links))
        self._apply_3dmodel()
    
    # -----------------------------------------------------------------------------------#
    def _apply_3dmodel(self):
        """
        Collect the corresponding 3D model for each link.\n
        Then get the relation between the DH transforms for each link and the pose of its corresponding 3D object
        """
        # current_path = os.path.abspath(os.path.dirname(__file__))
        self.links_3d = []
        for i in range(self.n + 1):
            file_name = None
            for ext in ['.stl', '.dae', '.ply']:
                if os.path.exists(os.path.join(self._link3D_dir, self.link3D_names[f'link{i}'] + ext)):
                    file_name = os.path.join(self._link3D_dir, self.link3D_names[f'link{i}'] + ext)
                    break                   
            if file_name is not None:
                if f'color{i}' in self.link3D_names:
                    self.links_3d.append(geometry.Mesh(file_name, color = self.link3D_names[f'color{i}']))
                else:
                    self.links_3d.append(geometry.Mesh(file_name,))
            else:
                raise ImportError(f'Cannot get 3D file at link {i}!')

        link_transforms = self._get_transforms(self._qtest)

        # Get relation matrix between the pose of the DH Link and the pose of the corresponding 3d object
        self._relation_matrices = [np.linalg.inv(link_transforms[i]) @ self._qtest_transforms[i] 
                                   for i in range(len(link_transforms))] 
        
    
    # -----------------------------------------------------------------------------------#
    def _update_3dmodel(self):
        """
        Update the robot's 3D model based on the relation matrices
        """
        link_transforms = self._get_transforms(self.q)
        for i, link in enumerate(self.links_3d):
            link.T = link_transforms[i] @ self._relation_matrices[i]
    
    # -----------------------------------------------------------------------------------#
    def _get_transforms(self,q):
        """
        Get the transform list represent each link
        """
        transforms = [self.base.A]
        L = self.links
        for i in range(self.n):
            transforms.append(transforms[i] @ L[i].A(q[i]).A)
        return transforms
    
    # -----------------------------------------------------------------------------------#
    def add_to_env(self, env):
        """
        Add the robot into a input Swift environment
        """
        if not isinstance(env, swift.Swift):
            raise TypeError('Environment must be Swift!')
        self._update_3dmodel()
        for link in self.links_3d:
            env.add(link)

    # Parameters Setter and Getter
    # -----------------------------------------------------------------------------------#
    def set_base(self, base):
        """ Set base for system based on user input """
        self.base = base * self.base
    
    def set_neutral_js(self, js):
        """ Set neutral joint states for system based on user input """
        self.neutral = js

    def get_ee_pose(self):
        """ Get end-effector pose of the robot """
        return self.fkine(self.q)

    def get_jointstates(self):
        """ Get robot joint states """
        return copy.deepcopy(self.q)
    
    # collision function:
    def collision_check(self, q, object : geometry.Mesh):
        """
        """
        # get the transforms of all links
        ee_pose = self.fkine(q)
        ee_sphere = geometry.Cylinder(0.03, self.d[self.n-1]/2,  pose = ee_pose)
        d, p1, p2 = ee_sphere.closest_point(object, 5)
        return d, p1, p2

    def grounded_check(self, q, ground_height = None):
        """
        """
        # if ground_height is not defined, use the base height as the ground height
        if ground_height is None:
            ground_height = self.base.A[2, 3]

        # get the transforms of all links
        link_transforms = self.get_link_poses(q)

        # check if all links are above the ground defined
        for j, link in enumerate(link_transforms):
            if j <= 1:
                continue
            elif link[2, 3] < ground_height +  0.02:
                return True
        return False 

    def get_ellipsoid(self):

        """ Get ellipsoid of the robot """

        ellipsoids = []
        for i in range(len(self.links)):
            
            # special define for major and minor axis for ellipsoid
            minor_axis = 0.03 if self.a[i] == 0 else copy.deepcopy(self.a[i]) / 2 + 0.02
            major_axis = copy.deepcopy(self.d[i]) / 2

            # define major and minor axis of ellipsoid
            ellipsoid = np.asarray([minor_axis, major_axis, minor_axis])

            ellipsoids.append(ellipsoid)

        if self.n == 6: # 7dof robot
            ellipsoids[6][2] = ellipsoids[6][1]
            ellipsoids[6][1] = ellipsoids[6][0]
        else: # 6dof robot
            ellipsoids[5][2] = ellipsoids[5][1]
            ellipsoids[5][1] = ellipsoids[5][0]

        return ellipsoids

    def get_link_poses(self, q=None):
        """
        :param q robot joint angles
        :param robot -  seriallink robot model
        :param transforms - list of transforms
        """
        if q is None:
            return self.fkine_all()
        return self.fkine_all(q)
    
        
    # -----------------------------------------------------------------------------------#
    def __setattr__(self, name, value):
        """
        Overload `=` operator so the object can update its 3D model whenever a new joint state is assigned
        """
        if name == 'q' and hasattr(self, 'q'):
            self._update_3dmodel()  # Update the 3D model before setting the attribute
        super().__setattr__(name, value)  # Call the base class method to set the attribute

