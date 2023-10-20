import os
import sys
import copy

import roboticstoolbox as rtb
import spatialgeometry as geometry
import spatialmath as sm
import spatialmath.base as smb

from swift import Swift

class WorkCell():

    _script_directory = os.path.dirname(os.path.abspath(__file__))

    def __init__(self, env : Swift, cell_location : sm.SE3):

        self._env = env
        self._center = cell_location

        # add human model
        self._human = self._add_model('worker.stl', sm.SE3(-0.341478, 0, 0), color = (1.0, 0.0, 0.0, 1.0))
        # self._cart = self._add_model('CART_ply.PLY', sm.SE3(x0.341478, 0, 0))

    def workcell_setup(self):

        self.desk = self._add_model('desk.stl', sm.SE3(0, 0, 0), color = (0.0, 0.0, 1.0, 1.0))
        self.container = self._add_model('container.stl', sm.SE3(0, 0, 0), color = (0.0, 0.0, 1.0, 1.0))
        self.printer = self._add_model('printer.stl', sm.SE3(0, 0, 0), color = (0.0, 0.0, 1.0, 1.0))
        self.printer2 = None

        self.estop1 = self._add_model('estop.stl', sm.SE3(0, 0, 0), color = (0.0, 0.0, 1.0, 1.0))
        self.estop2 = self._add_model('estop.stl', sm.SE3(0, 0, 0), color = (0.0, 0.0, 1.0, 1.0))
        self.estop3 = self._add_model('estop.stl', sm.SE3(0, 0, 0), color = (0.0, 0.0, 1.0, 1.0))
        self.power_supply = self._add_model('power_supply.stl', sm.SE3(0, 0, 0), color = (0.0, 0.0, 1.0, 1.0))
        self.fire_extinguisher = self._add_model('fire_extinguisher.stl', sm.SE3(0, 0, 0), color = (0.0, 0.0, 1.0, 1.0))


    def move_human(self, pose):
        """

        Args:
            pose (_type_): _description_
        """
        self._human.T = pose
        self._env.step(0)
        
    def get_human_pose(self):
        """
        Getter for human pose to use in work cell safety checking

        Returns:
            _type_: _description_
        """
        return copy.deepcopy(self._human.T)

    def _add_model(self, file_path, placement, color=None):
        """
        Add model to simulation environment
        Placement is relative pose of the object to cell center
        """
        model_full_path = os.path.join(self._script_directory, 'mesh', file_path)
        model_placement = self._center @ placement
        if color is None:
            model = geometry.Mesh(
                model_full_path, pose=model_placement,collision=True)
        else:
            model = geometry.Mesh(
                model_full_path, pose=model_placement,collision=True, color=color)

        self._env.add(model, collision_alpha=1)
        return model

