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
        self._human = self._add_model('worker.stl', sm.SE3(2.5, 1.5, 0), color = (1.0, 0.0, 0.0, 1.0)) # -0.341478
        self._cart = self._add_model('cart.dae', sm.SE3(0.5, 0, 0) @ sm.SE3.Rz(-90,'deg'))
        self._desk = self._add_model('desk.dae', sm.SE3(0, 0, 0) @ sm.SE3.Rz(180,'deg'))
        self._saywer_stand = self._add_model('saywer_stand.dae' , sm.SE3(0.55, 0.8, 0) @ sm.SE3.Rz(-90,'deg'))
        self._3dprinter_bed_0 = self._add_model('3dprinter_bed.dae',sm.SE3(-0.4, 0.9, 0.87))
        self._3dprinter_body_0 = self._add_model('3dprinter_body.dae', sm.SE3(-0.4, 0.9, 0.81))
        self._3dprinter_bed_1 = self._add_model('3dprinter_bed.dae',sm.SE3(-0.4, 0.4, 0.87))
        self._3dprinter_body_1 = self._add_model('3dprinter_body.dae', sm.SE3(-0.4, 0.4, 0.81))
        self._estop_0 = self._add_model('estop.dae', sm.SE3(0.86, -0.464, 0.81))
        self._estop_1 = self._add_model('estop.dae', sm.SE3(0.86, -1.1233, 0.81))
        self._estop_2 = self._add_model('estop.dae', sm.SE3(-0.75, 1.15, 0.81))
        self._tiles = self._add_model('tiles.dae', sm.SE3(0,0,-0.015))
        self._fire_extinguisher_0 = self._add_model('fire_extinguisher.dae', sm.SE3(1.135,-1.07,0))
        self._fire_extinguisher_1 = self._add_model('fire_extinguisher.dae', sm.SE3(-0.65,1.428,0) @ sm.SE3.Rz(90,'deg'))
        self._light_curtain_cyl = self._add_model('light_curtain_cyl.dae', sm.SE3(0, 0, 0) @ sm.SE3.Rz(180,'deg'))
        self._light_curtain_laser = self._add_model('light_curtain_laser.dae', sm.SE3(0, 0, 0) @ sm.SE3.Rz(180,'deg'))
        self._gate = self._add_model('gate.dae', sm.SE3(0, 0, 0) @ sm.SE3.Rz(180,'deg'))
        self._long3dprinting = self._add_model('long3dprinting.dae', sm.SE3(0, 0, 0) @ sm.SE3.Rz(180,'deg'))

    def get_cell_center(self):
        """
        Getter for cell center

        Returns:
            _type_: _description_
        """
        return copy.deepcopy(self._center)

    def move_human(self, pose):
        """
        Function to move human model in simulation

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

        Args:
            file_path (string): relative path to model file
            placement (spatialmath.SE3): relative pose of the object to the cell center
            color (list): list of RGBA values for the model
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
