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

import pygame
import sys
import time
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

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
        self.human_init_pos = sm.SE3(2.5, 1.5, 0)

        self.workcell_space = RectangularPrism(width=3.6, breadth=4, height=2,center= (self.cell_center @ sm.SE3(0,0,1)).A[0:3,3])
        self.vertices, self.faces, self.normals = self.workcell_space.get_data()
        
        self._human = self._add_model('worker.stl', self.human_init_pos, color = (1.0, 0.0, 0.0, 1.0)) # -0.341478
        self.human_thread = threading.Thread(target=self.move)
        
        self.human_status = 'SAFE'
        self._keyboard_status = False
        self._disable_keyboard = True
        
        # self._keys = None
        self.human_thread = threading.Thread(target=self.keyboard_move)
        self.human_thread.start()
        
        self._screen = None
        self._x = 0
        self._y = 0



        
        

    def enable_keyboard(self):
        pygame.init()
        window_width, window_height = 500, 50
        self._screen = pygame.display.set_mode((window_width, window_height))
        pygame.display.set_caption('Click Here!!!')
        self._font = pygame.font.Font('freesansbold.ttf', 18)
        self._text = self._font.render('Use Arrow Keys To Control Your Human!', True, (255, 255, 255))
        self._text_rect = self._text.get_rect()
        self._text_rect.center = (window_width // 2, window_height // 2)
        self._disable_keyboard = False
        self._keyboard_status = True
        
        
    def keyboard_move(self):
        speed = 3
        if self._keyboard_status:

            while True:
                try:
                    self._screen.fill((0,0,0))
                except:
                    pass
                
                try:
                    self._screen.blit(self._text, self._text_rect)
                except:
                    pass            
                
                try:
                    keys = pygame.key.get_pressed()
                except:
                    pass
                

                if keys[pygame.K_LEFT]:
                    # Update mesh transformation for left arrow key
                    self._y -= 0.01 * speed
                    # print('left')
                elif keys[pygame.K_RIGHT]:
                    # Update mesh transformation for right arrow key
                    self._y += 0.01 * speed
                    # print('right')
                    
                elif keys[pygame.K_UP]:
                    # Update mesh transformation for up arrow key
                    self._x -= 0.01 * speed
                    # print('up')
                    
                elif keys[pygame.K_DOWN]:
                    # Update mesh transformation for down arrow key
                    self._x += 0.01 * speed
                    # print('down')

                self.move(self._x, self._y)
                pygame.time.wait(30)
                try:
                    pygame.display.update()
                except:
                    pass
                
                if not self.disable_keyboard:
                    pygame.quit()
                    break
                
    
    def move(self, x, y):
        new_transform = sm.SE3(x,y,0)
        self._human.T = self.human_init_pos @ new_transform

        self._env.step(0.01)
        if self.is_in_workcell():
            self.human_status = 'DANGER'
        else: self.human_status = 'SAFE'

    def get_keyboard_status(self):
        return self._keyboard_status
    
    def disable_keyboard(self):
        self._keyboard_status = False
        self._disable_keyboard = True
        pygame.quit()
        # quit()

    def get_pose(self):  
        return copy.deepcopy(self._human.T)
    
    def human_state(self):
        return self.human_status
    
    def is_in_workcell(self):
        
        """
        Collision check using the closest point between the end-effector and the object,
        with offset line for additional 4 sides of the end-effector to ensure the boundary of near collision
        """
        result_face = False
        result_arm = False
        human_bound_face = 0.9
        human_bound_arm = 0.65
        
        human_line_face = {
            "start" : self.get_pose() @ smb.transl(-human_bound_face/2,0,0),
            "end" : self.get_pose() @ smb.transl(human_bound_face/4.5,0,0)
        }

        # Original line definition
        human_line_arm = {
            "start": self.get_pose() @ smb.transl(0, -human_bound_arm / 2, 0),
            "end": self.get_pose() @ smb.transl(0, human_bound_arm / 2, 0)
        }

        
        
        for j, face in enumerate(self.faces):
            vert_on_plane = self.vertices[face][0]
            intersect_p, check = line_plane_intersection(self.normals[j], 
                                                        vert_on_plane, 
                                                        human_line_face['start'][:3,3], 
                                                        human_line_face['end'][:3,3])
            triangle_list  = np.array(list(combinations(face,3)),dtype= int)
            if check == 1:
                for triangle in triangle_list:
                    if self._is_intersection_point_inside_triangle(intersect_p, self.vertices[triangle]):
                        result_face = True
        
        
        for j, face in enumerate(self.faces):
            vert_on_plane = self.vertices[face][0]
            intersect_p, check = line_plane_intersection(self.normals[j], 
                                                        vert_on_plane, 
                                                        human_line_arm['start'][:3,3], 
                                                        human_line_arm['end'][:3,3])
            triangle_list  = np.array(list(combinations(face,3)),dtype= int)
            if check == 1:
                for triangle in triangle_list:
                    if self._is_intersection_point_inside_triangle(intersect_p, self.vertices[triangle]):
                        result_arm = True
        
        result = False
        if result_face or result_arm:
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