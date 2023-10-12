from plate import Plate
import spatialgeometry as geometry
from spatialmath import SE3
import swift

env = swift.Swift()
env.launch(realtime= True)

pose = SE3.Trans(0,0,0)
plate = Plate(pose, env)

for i in range(100):
    pose *= SE3.Trans(0.1,0,0)
    plate.flat_plate_update(pose)
    env.step(0.1)
