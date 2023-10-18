from spatialmath import SE3
import spatialgeometry as geometry
import os
from math import pi
from swift import Swift

class Plate():
    """
    The frame of each segment is at its center
    """

    FULL_DIST = 0.22
    SEGMENT_NUM = 11 # odd num
    SEGMENT_LENGTH = FULL_DIST / SEGMENT_NUM

    def __init__(self, pose : SE3, env : Swift):
        """
        pose: the first segment's pose represents the whole plate's pose
        """
        self._env = env
        file_name = os.path.join(os.path.abspath(os.path.dirname(__file__)),'mesh','plate.dae')
        
        self._segments = [geometry.Mesh(file_name, color = (1,95/255,31/255,1)) for i in range(Plate.SEGMENT_NUM)]
        
        self.flat_plate_update(pose)
        self.add_to_env()
        

    def set_pose(self, pose):
            self._mesh.T = pose

    def flat_plate_update(self, pose : SE3):
        seg_pose = pose
        for seg in self._segments:
            seg.T = seg_pose
            seg_pose *= SE3.Trans(Plate.SEGMENT_LENGTH,0,0)

    def add_to_env(self):
        for seg in self._segments:
            self._env.add(seg)

    def bend(self, i):

        # The anchor point stays staionary during bending
        anchor_segment_number = int((Plate.SEGMENT_NUM-1)/2) #5
        anchor_point = self._segments[anchor_segment_number].T 

        # Theta is the orientation of each segment
        step = 10

        # Increment is the increment of theta after each step
        INCREMENT = pi/30 * i

        theta = anchor_point
        for seg in self._segments[(anchor_segment_number+1):]:
            theta = theta * SE3.Trans(-Plate.SEGMENT_LENGTH/2,0,0) * SE3.Ry(-INCREMENT) * SE3.Trans(-Plate.SEGMENT_LENGTH/2,0,0)
            seg.T = theta         

        theta = anchor_point
        for seg in self._segments[:anchor_segment_number]:

            theta = theta * SE3.Trans(Plate.SEGMENT_LENGTH/2,0,0) * SE3.Ry(INCREMENT) * SE3.Trans(Plate.SEGMENT_LENGTH/2,0,0)
            seg.T = theta      


if __name__ == "__main__":
    pass

    