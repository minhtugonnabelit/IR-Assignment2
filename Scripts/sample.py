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
Tep = ur.fkine(ur.q) @ sm.SE3.Rz(np.pi/6) @ sm.SE3.Ry(np.pi/6)
env.add(ur)

# time = 2
# step = 50
# time_step = time/step
# path = rtb.ctraj(current_ee, Tep, t = step)

# # x = np.zeros((2,step))
# # s = rtb.trapezoidal(0,1,step)
# # for i in range(step):
# #     x[:,i] = current_ee
 
# for i in range(len(path)-1):

#     current_ee_pose = ur.fkine(ur.q).A
#     cur_ee_pos = current_ee_pose[0:3,3]
#     # cur_ee_ori = smb.tr2rpy(current_ee_pose[0:3,0:3], order='xyz')

#     # print(path[i+1])
#     desired_ee_pos = path[i+1].A[0:3,3]
#     # desired_ee_ori = smb.tr2rpy(path[i].A[0:3,0:3], order='xyz')
#     angular_vel = np.zeros(3)

#     # lin_vel = (path[i+1][0:3,3] - path[i][0:3,3]) / time_step

#     # get linear velocity between interpolated point and current pose of ee
#     lin_vel = (desired_ee_pos - cur_ee_pos) / time_step

#     # get angular velocity between interpolated ...
#     # ang_vel = (desired_ee_ori - cur_ee_ori) / time_step

#     ee_vel = np.hstack((lin_vel, angular_vel))

#     joint_vel = np.linalg.pinv(ur.jacob0(ur.q)) @ np.transpose(ee_vel )
#     # print(np.linalg.pinv(ur.jacobe(ur.q))[:,0:3])
#     # print(joint_vel)
    
#     current_js = copy.deepcopy(ur.q)
#     # q = current_js + joint_vel * time_step
#     # print(q)
#     # ur.q = q
#     ur.qd = joint_vel
#     env.step(time_step)

def rmrc_straight_line(robot : rtb.DHRobot, desired_pose : sm.SE3, time=1):

    step = 100
    time_step = time/step
    current_ee_pose = robot.fkine(robot.q)
    # height_diff = current_ee_pose.A[2,3] - desired_pose.A[2,3]
    # pose_toward = smb.transl(0,0,-height_diff) @ current_ee_pose.A
    # path = rtb.ctraj(current_ee_pose, sm.SE3(pose_toward), t = step)
    path = rtb.ctraj(current_ee_pose,desired_pose, t = step)

    # s = rtb.trapezoidal(0, 1, step).q
    # # print(s)                                                                   # Create the scalar function
    # path = []                                                                  # Create memory allocation for variables
    # for i in range(step):
    #     point = (1 - s[i]) * current_ee_pose + s[i] * desired_pose
    #     print(point)
    #     path.append(sm.SE3(point)) 

    for i in range(len(path)-1):

        prev_ee_pos = path[i].A[0:3,3]
        desired_ee_pos = path[i+1].A[0:3,3]

        # get linear velocity between interpolated point and current pose of ee
        lin_vel = (desired_ee_pos - prev_ee_pos) / time_step

        # get angular velocity between interpolated ...
        s_omega = (path[i+1].A[0:3,0:3] @ np.transpose(robot.fkine(robot.q).A[0:3,0:3]) - np.eye(3)) / time_step
        ang_vel = [s_omega[2,1], s_omega[0,2], s_omega[1,0]]

        # combine vel
        ee_vel = np.hstack((lin_vel, ang_vel))

        # get joint velocities 
        joint_vel = np.linalg.pinv(robot.jacob0(robot.q)) @ np.transpose(ee_vel)
        
        current_js = copy.deepcopy(robot.q)
        # q = current_js + joint_vel * time_step
        robot.qd = joint_vel
        # robot.q = q
        env.step(time_step)

rmrc_straight_line(ur, Tep, time=2)

Tep2 = ur.fkine(ur.q) @ sm.SE3.Tz(-0.1)
rmrc_straight_line(ur, Tep2, time=2)
