from plate import Plate
# from ..plate
import spatialgeometry as geometry
from spatialmath import SE3
import swift
import os

env = swift.Swift()
env.launch(realtime= True)

pose = SE3.Trans(0,0,0)
plate = Plate(pose, env)

# TEST 1 : Moving plate
# for i in range(100):
#     pose *= SE3.Trans(0.01,0,0) * SE3.Rz(0.1)
#     plate.move_flat_plate(pose)
#     env.step(0.1)
#     print(i)

# env.hold()


# TEST 2 : Bending plate
steps = 30
all_seg = []

for i in range(steps):
    seg_array = []
    plate.bend(i, seg_array)
    all_seg.append(seg_array)
    env.step(0.1)

for seg_array in reversed(all_seg):
    plate.unbend(seg_array)
    env.step(0.1)

env.hold()
