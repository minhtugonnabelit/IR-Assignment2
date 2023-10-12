from roboticstoolbox import *
from spatialmath import SE3
from math import pi
# import numpy as np
 

# left = rtb.models.DH.Baxter('left') # Baxter left arm
# right = rtb.models.DH.Baxter('right') # Baxter right arm
# left.base = SE3(0.064614, 0.25858, 0.119) * SE3.Rx(pi / 4)
# right.base = SE3(0.063534, -0.25966, 0.119) * SE3.Rx(-pi / 4)

# qLeft = [pi/10,0,0,0,0,0,pi/10] 
# qRight = [-pi/10,0,0,0,0,0,-pi/10]

# left_pos = left.fkine(qLeft)
# right_pos = right.fkine(qRight)


# dist = np.linalg.norm(left_pos.A[0:3,3] - right_pos.A[0:3,3])
# print(dist)



import ir_support as ir
import numpy as np



# link1 = DHLink(d= 0, a= 1, alpha=0, qlim= [-pi, pi]) 
# link2 = DHLink(d= 0, a= 1, alpha=0, qlim= [-pi, pi]) 
# link3 = DHLink(d= 0, a= 1, alpha=0, qlim= [-pi, pi]) 
# link4 = DHLink(d= 0, a= 1, alpha=0, qlim= [-pi, pi]) 
# link5 = DHLink(d= 0, a= 1, alpha=0, qlim= [-pi, pi]) 


# robot = DHRobot([link1, link2, link3, link4, link5], name= 'myRobot')
# workspace = [-2, 2, -2, 2, -2, 2]
# # q =  np.zeros([1,3])
# # q = [0,pi/10,-pi/2,0,pi/    ,0]
# q = [45,-45,40,-45,0] 
# q = np.deg2rad(q)
#      # robot.plot(q= q, limits= workspace)
# tr = robot.fkine(q).A
# print(tr)
# q1 = np.array([pi/10, pi/7, pi/5, pi/3, pi/4, pi/6])
# q2 = np.array([-pi/6, -pi/3, -pi/4, -pi/8, -pi/7, -pi/10])

# def max_joint_vel( q1, q2, steps):

#     s = trapezoidal(0, 1, steps).q                                                                      # Create the scalar function
#     q_matrix = np.empty((steps, 6))                                                                     # Create memory allocation for variables
#     for i in range(steps):
#         q_matrix[i, :] = (1 - s[i]) * q1 + s[i] * q2

#     velocity = np.zeros([steps, 6])
#     for i in range(1,steps):
#         velocity[i,:] = q_matrix[i,:] - q_matrix[i-1,:]

#     velocity = np.round(velocity, decimals= 4)
#     print(np.round(velocity, decimals= 4))
#     return np.max(abs(velocity))

# print("return: ", max_joint_vel(q1, q2, 55))

import roboticstoolbox as rtb
from math import pi
a = [0, 0.4318, 0.0203, 0, 0, 0]
d = [0, 0, 0.15005, 0.4318, 0, 0]
alpha = [pi/2, 0, -pi/2, pi/2, -pi/2, 0]
qlim = [[-2.7925, 2.7925],[-0.7854, 3.9270],[-3.9270, 0.7854],[-1.9199, 2.9671],[-1.7453, 1.7453],[-4.6426, 4.6426]]
links = []

for i in range(6):
    link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim= qlim[i])
    links.append(link)
p560 = rtb.DHRobot(links, name ='Puma560')

# Use the ikine_LM
end = SE3(0.7,0.1,0.2)
qn = [0, 0.78539816, 3.14159265, 0, 0.78539816, 0]

q = p560.ikine_LM(end, q0 = qn, mask = [1,1,1,0,0,0])
print(q)