import roboticstoolbox as rtb
import spatialmath as sm
import spatialmath.base as smb
import numpy as np
from swift import Swift
from Sawyer_model.sawyer import Sawyer

import ipdb
import copy

# Make and instance of the Swift simulator and open it
env = Swift()
env.launch(realtime=True)

robot = Sawyer(env)
robot.update_sim()

# robot motion test with joint space
robot.test()

# robot motion test with Cartesian space
pose1 = robot.get_ee_pose() @ sm.SE3.Tz(-0.2)
robot.go_to_CartesianPose(pose1, time=2)

# test motion function with rotating end-effector pi/3 rad around its local z axis
robot.rotate_head(np.pi/3)

# test motion function with forwarding end-effector -0.5m in its local x direction
pose2 = robot.get_ee_pose() @ sm.SE3.Tx(-0.5)
robot.go_to_CartesianPose(pose2, time=2)
robot.home()



























# def rmrc_straight_line(robot : rtb.DHRobot, desired_pose : sm.SE3, time=1):

#     step = 100
#     time_step = time/step
#     current_ee_pose = robot.fkine(robot.q)

#     path = rtb.ctraj(current_ee_pose,desired_pose, t = step)

#     for i in range(len(path)-1):

#         prev_ee_pos = path[i].A[0:3,3]
#         desired_ee_pos = path[i+1].A[0:3,3]

#         # get linear velocity between interpolated point and current pose of ee
#         lin_vel = (desired_ee_pos - prev_ee_pos) / time_step

#         # get angular velocity between interpolated ...
#         s_omega = (path[i+1].A[0:3,0:3] @ np.transpose(robot.fkine(robot.q).A[0:3,0:3]) - np.eye(3)) / time_step
#         ang_vel = [s_omega[2,1], s_omega[0,2], s_omega[1,0]]

#         # combine vel
#         ee_vel = np.hstack((lin_vel, ang_vel))

#         # get joint velocities 
#         joint_vel = np.linalg.pinv(robot.jacob0(robot.q)) @ np.transpose(ee_vel)
        
#         current_js = copy.deepcopy(robot.q)
#         q = current_js + joint_vel * time_step
#         robot.q = q
#         env.step(time_step)
