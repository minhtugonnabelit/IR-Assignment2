from plate import Plate
# from ..plate
import spatialgeometry as geometry
from spatialmath import SE3
import swift
import os

import numpy as np
import time

env = swift.Swift()
env.launch(realtime= True)


bunny_location = SE3(0.03, 0, 0) * SE3.Rz(np.pi/2)
pose = SE3.Trans(0,0,0.1) * SE3.Rx(np.pi/3)
plate = Plate(pose, env, bunny_location= bunny_location)

# TEST 1 : Moving plate
# for i in range(100):
#     pose *= SE3.Trans(0.01,0,0) * SE3.Rz(0.1)
#     plate.move_flat_plate(pose)
#     env.step(0.1)
#     print(i)

# env.hold()

ax1 = geometry.Axes(0.2,pose=pose@SE3(0.11,0,0))
ax2 = geometry.Axes(0.2,pose=pose@SE3(-0.11,0,0))

env.add(ax1)
env.add(ax2)

time.sleep(1)

# TEST 2 : Bending plate
steps = 30
all_seg = []

step_bunny = 0.015 / steps

for i in range(steps):
    seg_array = []
    pick, bend = plate.bend(i, seg_array)
    ax1.T = pick
    ax2.T = bend
    all_seg.append(seg_array)
    plate.drop_bunny_step(step= step_bunny * i)
    
    env.step(0.0)
    time.sleep(0.01)


# for i in range(steps):
#     plate.drop_bunny_step(step= step_bunny * i)
#     env.step(0.05)

for seg_array in reversed(all_seg):
    pick, bend = plate.unbend(seg_array)
    ax1.T = pick
    ax2.T = bend
    env.step(0.1)


env.hold()
