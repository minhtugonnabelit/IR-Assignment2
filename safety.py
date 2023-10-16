import copy

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import spatialmath as sm
import spatialmath.base as smb
import spatialgeometry as geometry
from itertools import combinations


import roboticstoolbox as rtb
from ir_support import make_ellipsoid, line_plane_intersection, RectangularPrism

from robot.m_DHRobot3D import M_DHRobot3D
from robot.sawyer import Sawyer
from robot.astorino import Astorino

# import for testing purpose
from swift import Swift, Slider
import time

class Safety:

    def __init__(self, robot : M_DHRobot3D) -> None:

        self.robot = robot

        # get ellipsoid parameters (x,y,z) with major and minor axis allocated to a and d respectively
        self._ellipsoids = self.get_ellipsoid()

        # generate mesh points for each ellipsoid in the local frame of each link
        self._ellipsoids_meshlist = self.get_ellipsoid_meshlist()

    def get_link_poses(self, q=None):
        """
        :param q robot joint angles
        :param robot -  seriallink robot model
        :param transforms - list of transforms
        """
        if q is None:
            return self.robot.fkine_all()
        return self.robot.fkine_all(q)
    
    def get_link_centers(links_tf):

        """
        Get the center of each link"""

        links_center = []
        for i in range(len(links_tf)-1):

            # keep orientation of end of link
            tf = copy.deepcopy(links_tf[i+1].A)

            # set the center of the link to be the average of the end of link and the start of link
            tf[0:3,3] = (links_tf[i+1].A[0:3,3] + links_tf[i].A[0:3,3]) / 2
            links_center.append(tf)

        return links_center

    def get_ellipsoid(self):

        """ Get ellipsoid of the robot """

        ellipsoids = []
        thickness = 0.05
        for i in range(len(self.robot.links)):
            
            # special define for major and minor axis for ellipsoid
            minor_axis = thickness if self.robot.a[i] == 0 else copy.deepcopy(self.robot.a[i]) / 2 + thickness
            major_axis = copy.deepcopy(self.robot.d[i]) / 2

            # define major and minor axis of ellipsoid
            ellipsoid = np.asarray([minor_axis, major_axis, minor_axis])

            ellipsoids.append(ellipsoid)

        if self.robot.n == 7:   # 7dof robot
            ellipsoids[6][2] = ellipsoids[6][1]
            ellipsoids[6][1] = ellipsoids[6][0]
        else:                   # 6dof robot
            ellipsoids[5][2] = ellipsoids[5][1]
            ellipsoids[5][1] = ellipsoids[5][0]

        return ellipsoids
    
    def get_ellipsoid_meshlist(self):
        
        """ Get the ellipsoid mesh points corresponded to each link stick to their local frame """

        meshlist = []

        for ellipsoid in self._ellipsoids:
            meshlist.append(Safety._make_ellipsoid(ellipsoid, np.zeros(3), density=(40,20)))

        return meshlist   
    
    def transform_ellipsoid(self, links_center):

        # get the ellipsoid mesh points corresponded to each link and transform to the world frames
        ellip_transforms = []
        for i in range(len(self._ellipsoids_meshlist)):
            
            ellip = []
            mesh = np.array(self._ellipsoids_meshlist[i])

            # Reshaping the mesh points to be a two-dimensional array and adding a row of ones for homogeneous coordinates
            homogeneous_mesh_points = np.vstack((mesh.reshape(3, -1), np.ones((1, mesh.shape[1] * mesh.shape[2]))))
            
            # Applying the transformation to the mesh points with corresponding link centers for mesh points relative to world frame
            transformed_points = links_center[i] @ homogeneous_mesh_points

            # normalize the homogenous part of the transformed points
            w = transformed_points[-1, :]
            normalized_points = transformed_points / w

            # Removing the last row to return to (x, y, z, 1) coordinates to support computation optimization
            ellip_transforms.append(normalized_points)
        
        return ellip_transforms

    # -------------------------------------------------------------------#
    ######################################################################
    # MAINTAINING
    # def object_collision_check(self, q, vecteces, faces, normals):

    #     """
    #     Standard version using line-plane intersection 
    #     # MAINTAINING"""

    #     links_tf = self.get_link_poses(q)


    #     return False
    
    # ## so called most stable collision check up to now
    # def is_collision(self, q_matrix, faces, vertex, face_normals, return_once_found = True):
    #     """
    #     This is based upon the output of questions 2.5 and 2.6
    #     Given a robot model (robot), and trajectory (i.e. joint state vector) (q_matrix)
    #     and triangle obstacles in the environment (faces,vertex,face_normals)
    #     """
    #     result = False

    #     for i, q in enumerate(q_matrix):

    #         # Get the transform of every joint (i.e. start and end of every link)
    #         tr = self.get_link_poses(q)
            
    #         # Go through each link and also each triangle face
    #         for i in range(np.size(tr,2)-1):
    #             for j, face in enumerate(faces):
    #                 vert_on_plane = vertex[face][0]
    #                 intersect_p, check = line_plane_intersection(face_normals[j], 
    #                                                             vert_on_plane, 
    #                                                             tr[i][:3,3], 
    #                                                             tr[i+1][:3,3])
    #                 # list of all triangle combination in a face
    #                 triangle_list  = np.array(list(combinations(face,3)),dtype= int)
    #                 if check == 1:
    #                     for triangle in triangle_list:
    #                         if self._is_intersection_point_inside_triangle(intersect_p, vertex[triangle]):

    #                             result = True
    #                             if return_once_found:
    #                                 return result
    #                             break
    #     return result

    # -------------------------------------------------------------------#
    ######################################################################

    # collision function:
    def collision_check(self, q, object : geometry.Mesh):

        """
        Cheating methods using the closest point between the end-effector and the object, 
        which is used to apply the automated damper for collision avoidance in the controller """
        
        # get the transforms of all links
        ee_pose = self.robot.fkine(q)

        # map a virtual sphere to the end-effector 
        ee_sphere = geometry.Cylinder(0.03, self.robot.d[self.robot.n-1]/2,  pose = ee_pose)

        # check if the closest point between the end-effector and the object is within the virtual sphere
        return ee_sphere.closest_point(object, 5)
    
    ## self collision check with ellipsoid mesh extracted for each link
    # -------------
    def is_self_collided(self, q):
        """
        Self collision check using ellipsoid.
        Principle is from extracting the ellipsoid mesh points corresponded to each link, 
        then transform its to each link ceener to check if there is any intersection
        """

        # get the transforms of all links end and center
        links_tf = self.get_link_poses(q)
        links_center = Safety.get_link_centers(links_tf)

        # get the ellipsoid mesh points corresponded to each link and transform to the world frames
        ellip_transforms = self.transform_ellipsoid(links_center)

        # iteration through each link
        for i, center in enumerate(links_center):

            # iteration through each link but avoid the currentt link and neighbor links
            for j in range(len(links_center)):

                if abs(j-i) <= 1:
                    continue
            
                # iteration through each ellipsoid mesh points
                for point in np.transpose(ellip_transforms[j][:]):
                         
                    # transform the extracted ellipsoid mesh points to the local frame of the current link
                    transformed_point = np.linalg.inv(center) @ point

                    # return once collision is detected
                    if np.sum(transformed_point[0:3]**2 / self._ellipsoids[i]**2) <= 1:
                        print(j)
                        return True

        return False
    
    ## function to check if the robot is hit the ground
    def grounded_check(self, q, ground_height = None):

        """
        Function to check if any link of the robot is hit the ground height defined """

        # if ground_height is not defined, use the base height as the ground height
        if ground_height is None:
            ground_height = self.robot.base.A[2, 3]

        # get the transforms of all links
        link_transforms = self.get_link_poses(q).A

        # check if all links are above the ground defined
        for j, link in enumerate(link_transforms):
            if j <= 1:
                continue
            elif link[2, 3] < ground_height +  0.02:
                return True
        return False 
    

    # ---------------------------------------------------------------------------------------#
    # EXTRACTED FUNCTION FROM IR-SUPPORT LIBRARY WITH MINOR CHANGES to fit the use
    def _is_intersection_point_inside_triangle(intersect_p, triangle_verts):
        u = triangle_verts[1, :] - triangle_verts[0, :]
        v = triangle_verts[2, :] - triangle_verts[0, :]

        uu = np.dot(u, u)
        uv = np.dot(u, v)
        vv = np.dot(v, v)

        w = intersect_p - triangle_verts[0, :]
        wu = np.dot(w, u)
        wv = np.dot(w, v)

        D = uv * uv - uu * vv

        # Get and test parametric coords (s and t)
        s = (uv * wv - vv * wu) / D
        if s < 0.0 or s > 1.0:  # intersect_p is outside Triangle
            return 0

        t = (uv * wu - uu * wv) / D
        if t < 0.0 or (s + t) > 1.0:  # intersect_p is outside Triangle
            return False

        return True  # intersect_p is in Triangle

    def _make_ellipsoid(ellipsoid_info, center, u=None, v=None, density=(10,5)):

        """
        Simple custom function to create an ellipsoid.
        
        :param ellipsoid_info: 1x3 array-like (ellipsoid radii) or 3x3 array (ellipsoid matrix)
        :param center: center of the ellipsoid
        :param color: color for ellipsoid
        :param u: list of azimuthal angle in spherical coordinates (optional)
        :param v: list of polar angle in spherical coordinates (optional)
        :param density: density of the ellipsoid surface (optional)
        :param ax: axis to plot on (optional)
        :return surface object & tuple of mesh data (X, Y, Z)
        """

        if np.shape(ellipsoid_info) == (1, 3) or np.shape(ellipsoid_info) == (3,):
            lengths = ellipsoid_info
            eigenvectors = np.eye(3)
        elif np.shape(ellipsoid_info) == (3, 3):
            eigenvalues, eigenvectors = np.linalg.eig(ellipsoid_info)
            lengths = np.sqrt(np.abs(eigenvalues))
        else:
            raise ValueError('Invalid input ellipsoid info!')

        # Ellipsoid surface
        if u is None:
            u = np.linspace(0, 2 * np.pi, density[0])
        if v is None:
            v = np.linspace(0, np.pi, density[1])

        # Generate surface points of the ellipsoid
        X = []
        Y = []
        Z = []
        for phi in u:
            for theta in v:
                point = np.array([
                    lengths[0] * np.cos(phi) * np.sin(theta),
                    lengths[1] * np.sin(phi) * np.sin(theta),
                    lengths[2] * np.cos(theta)
                ])
                rotated_point = np.dot(eigenvectors, point)
                X.append(rotated_point[0] + center[0])
                Y.append(rotated_point[1] + center[1])
                Z.append(rotated_point[2] + center[2])

        X = np.array(X).reshape(len(u), len(v))
        Y = np.array(Y).reshape(len(u), len(v))
        Z = np.array(Z).reshape(len(u), len(v))
    
        return (X,Y,Z)
    
    def test_ellipsoid_mesh(self):

        fig = self.robot.plot(q=sawyer.neutral, block=False, )
        ax = fig.ax

        # get the transforms of all links
        links_tf = self.get_link_poses(self.robot.neutral)

        # get the center of each link
        links_center = Safety.get_link_centers(links_tf)

        # get the ellipsoid mesh points corresponded to each link and transform to the world frames
        ellip_transforms = sawyer_safety.transform_ellipsoid(links_center)

        # eliminate the last column of the ellipsoid mesh points
        ellip_transforms = np.array(ellip_transforms)[:,:-1, :]

        for ellip in ellip_transforms:
            ax.scatter(ellip[0,:], ellip[1,:], ellip[2,:], c='r', s=1)

        # plot the ellipsoid mesh points in the world frame
        fig.hold()


    

## TESTING SPACE
if __name__ =='__main__':

    env = Swift()
    env.launch()

    sawyer = Sawyer(env)
    sawyer_safety = Safety(sawyer)

    # sawyer = Astorino(env)
    # sawyer_safety = Safety(sawyer)
    sawyer_safety.test_ellipsoid_mesh()
    

        
        