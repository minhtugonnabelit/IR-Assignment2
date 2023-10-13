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

# for i in range(100):
#     pose *= SE3.Trans(0.01,0,0)
#     plate.flat_plate_update(pose)
#     env.step(0.1)
#     print(i)

# env.hold()

for i in range(10):
    plate.bend(i)
    env.step(1)

env.hold()
