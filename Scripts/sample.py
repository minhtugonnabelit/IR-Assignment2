import roboticstoolbox as rtb
import spatialmath as sm
import spatialmath.base as smb
import numpy as np
from swift import Swift

import ipdb
import copy

# Make and instance of the Swift simulator and open it
env = Swift()
env.launch(realtime=True)

# Make a panda model and set its joint angles to the ready joint configuration
ur = rtb.models.UR3()
ur.q = [0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0 ]
current_ee = ur.fkine(ur.q)
Tep = sm.SE3.Tz(-0.2) @ ur.fkine(ur.q)
env.add(ur)

time = 2
step = 50
time_step = time/step
path = rtb.ctraj(current_ee, Tep, t = step)

# x = np.zeros((2,step))
# s = rtb.trapezoidal(0,1,step)
# for i in range(step):
#     x[:,i] = current_ee
 
for i in range(len(path)-1):

    current_ee_pose = ur.fkine(ur.q).A
    cur_ee_pos = current_ee_pose[0:3,3]
    # cur_ee_ori = smb.tr2rpy(current_ee_pose[0:3,0:3], order='xyz')

    # print(path[i+1])
    desired_ee_pos = path[i+1].A[0:3,3]
    # desired_ee_ori = smb.tr2rpy(path[i].A[0:3,0:3], order='xyz')
    angular_vel = np.zeros(3)

    # lin_vel = (path[i+1][0:3,3] - path[i][0:3,3]) / time_step

    # get linear velocity between interpolated point and current pose of ee
    lin_vel = (desired_ee_pos - cur_ee_pos) / time_step

    # get angular velocity between interpolated ...
    # ang_vel = (desired_ee_ori - cur_ee_ori) / time_step

    ee_vel = np.hstack((lin_vel, angular_vel))

    joint_vel = np.linalg.pinv(ur.jacob0(ur.q)) @ np.transpose(ee_vel )
    # print(np.linalg.pinv(ur.jacobe(ur.q))[:,0:3])
    # print(joint_vel)
    
    current_js = copy.deepcopy(ur.q)
    # q = current_js + joint_vel * time_step
    # print(q)
    # ur.q = q
    ur.qd = joint_vel
    env.step(time_step)

    # print(pose_diff)
# print(path)

# Set a desired and effector pose an an offset from the current end-effector pose

# print(ur.fkine(ur.q))

# Add the robot to the simulator

# # Simulate the robot while it has not arrived at the goal
# arrived = False
# while not arrived:

#     # Work out the required end-effector velocity to go towards the goal
#     v, arrived = rtb.p_servo(ur.fkine(ur.q), Tep, 1)
    
#     # Set the Panda's joint velocities
#     ur.qd = np.linalg.pinv(ur.jacobe(ur.q)) @ v
    
#     # Step the simulator by 50 milliseconds
#     env.step(0.01)