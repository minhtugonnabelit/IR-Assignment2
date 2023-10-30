import spatialgeometry as geometry
import spatialmath.base as smb
import spatialmath as sm

import threading
import copy
import os
import numpy as np
from itertools import combinations

from rectangularprism import RectangularPrism
from ir_support import line_plane_intersection

class Human():
    
    _script_directory = os.path.dirname(os.path.abspath(__file__))


    """NEED TO BUILD HUMAN MOVE BY SLIDERS OR KEYBOARDS"""
    def __init__(self, env, cell_center, spawn_location) -> None:
        """_summary_

        Args:
            env (_type_): :3===>
            cell_center (sm.SE3): cell center
            spawn_location (sm.SE3): human location relative to cell center
        """
        
        self._env = env
        self.cell_center = cell_center

        self.workcell_space = RectangularPrism(width=3.6, breadth=4, height=2, center=self.cell_center @ sm.SE3(0,0,1))
        self.vertices, self.faces, self.normals = self.workcell_space.get_data()
        
        self._human = self._add_model('worker.stl', sm.SE3(2.5, 1.5, 0), color = (1.0, 0.0, 0.0, 1.0)) # -0.341478
        self.human_thread = threading.Thread(target=self._move)
        
    def _move(self):
        pass 
    
    def get_pose(self):
        
        return copy.deepcopy(self._human.T)
    
    def is_in_workcell(self):
        
        """
        Collision check using the closest point between the end-effector and the object,
        with offset line for additional 4 sides of the end-effector to ensure the boundary of near collision
        """
        result = False
        human_bound = 0.5
        human_line = {
            "start" : self.get_pose() @ sm.SE3(-human_bound/2,0,0),
            "end" : self.get_pose() @ sm.SE3(human_bound/2,0,0)
        }

        for j, face in enumerate(self.faces):
            vert_on_plane = self.vertices[face][0]
            intersect_p, check = line_plane_intersection(self.normals[j], 
                                                        vert_on_plane, 
                                                        human_line['start'][:3,3], 
                                                        human_line['end'][:3,3])
            triangle_list  = np.array(list(combinations(face,3)),dtype= int)
            if check == 1:
                for triangle in triangle_list:
                    if self._is_intersection_point_inside_triangle(intersect_p, self.vertices[triangle]):
                        result = True
        return result
        
            
    def _add_model(self, file_path, placement, color=None):
        """
        Add model to simulation environment
        Placement is relative pose of the object to cell center

        Args:
            file_path (string): relative path to model file
            placement (spatialmath.SE3): relative pose of the object to the cell center
            color (list): list of RGBA values for the model
        """
        model_full_path = os.path.join(self._script_directory, 'mesh', file_path)
        model_placement = self.cell_center @ placement
        if color is None:
            model = geometry.Mesh(
                model_full_path, pose=model_placement,collision=True)
        else:
            model = geometry.Mesh(
                model_full_path, pose=model_placement,collision=True, color=color)


        self._env.add(model, collision_alpha=1)
        return model
    
    
    # ---------------------------------------------------------------------------------------#
    # EXTRACTED FUNCTION FROM IR-SUPPORT LIBRARY WITH MINOR CHANGES to fit the use
    def _is_intersection_point_inside_triangle(self, intersect_p, triangle_verts):
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