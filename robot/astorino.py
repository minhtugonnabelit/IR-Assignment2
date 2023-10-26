## @file
#  @brief Astorino Kawasaki Robot defined by standard DH parameters with 3D model
#  @author Long Thinh Le
#  @date Oct 5, 2023


import roboticstoolbox as rtb
import spatialmath as sm
import os
import numpy as np
import spatialmath.base as smb
import spatialgeometry as geometry
from spatialmath.base import *
from spatialmath import SE3
import time

from swift import Swift
from robot.m_DHRobot3D import M_DHRobot3D
# from m_DHRobot3D import M_DHRobot3D

class Astorino(M_DHRobot3D):
    
    _NEUTRAL = [0, -np.pi/2, 0, 0, 0, 0]
    _script_directory = os.path.dirname(os.path.abspath(__file__))

    def __init__(
        self,
        env: Swift,
        base = sm.SE3(0,0,0),
        gripper_ready = False, # Indicate for mounting gripper
        gui = None
    ):
        # DH links
        links = self._create_DH() # need to work on function
        self._gripper_ready = gripper_ready
        self._env = env
        
        # Names of the robot link files in the directory
        link3D_names = dict(
            link0 = 'Astorino_base',
            link1 = 'Astorino_Robot_Joint_1',
            link2 = 'Astorino_Robot_Joint_2',
            link3 = 'Astorino_Robot_Joint_3',
            link4 = 'Astorino_Robot_Joint_4',
            link5 = 'Astorino_Robot_Joint_5',
            link6 = 'Astorino_Robot_Joint_6'
            )
        
        # A joint config and the 3D object transforms to match that config
        qtest = [0,-np.pi/2,0,0,0,0]
        qtest_transforms = [
            smb.transl(0,0,0),
            smb.transl(0,0,0.0555),
            smb.transl(0.0635,0,0.16598),
            smb.transl(0.0635,0,0.38798),
            smb.transl(0.064,0,0.45398),
            smb.transl(0.350,0,0.45398),
            smb.transl(0.350,0,0.494)
        ]
        
        current_path = os.path.abspath(os.path.dirname(__file__)) # Obtain the absolute path of the directory containing the current Python script. "__file__": builtin variable in Python that represents the current path when execute script
        # For example: Python file in '/home/user/projects/my_script.py' then current_path will be assign '/home/user/projects/'
        
        current_path = os.path.join(current_path, "Astorino_model")
        
        super().__init__( # calling the constructor of a parent class to initialize an object of the class.
            links,
            link3D_names,
            name="Astorino",
            link3d_dir = current_path,
            qtest = qtest,
            qtest_transforms= qtest_transforms,
        )
        
        self.base = base * self.base
        self.q = qtest
        self.set_neutral_js(self._NEUTRAL)

        # Add gripper
        self.gripper = AstorinoGripper(base= self.fkine(self.q))
        self.gripper_offset = sm.SE3(0,0,0.13)
        # self.ax = geometry.Axes(0.2, pose=self.fkine(self.q) @ self.gripper_offset)

        self.update_sim()


    def _create_DH(self):
        """
        Creat Astorino's DH parameter
        """
        mm = 1e-3
        number_of_links = 6
        
        # Kinematics Parameters
        # 63.5 start from link 2 (base: no, link 1: no)

        a = np.r_[63.5,  222, 66,     0,  0,  0]*mm
        d = np.r_[165.98,  0,  0, 286.5,  0,  0]*mm
        
        alpha = [-np.pi/2, 0, -np.pi/2, np.pi/2, np.pi/2, 0]
        offset = [0,0,0,0,np.pi/2,0]
        
        qlim = [[-2*np.pi, 2*np.pi] for _ in range(number_of_links)]
        
        links = []
        
        for j in range(number_of_links):
            link = rtb.RevoluteDH(
                d=d[j], a=a[j], alpha=alpha[j], offset=offset[j], qlim=qlim[j])
            links.append(link)
    
        return links
        
    def update_sim(self):
        """ 
        Update simulation
        """
        self.add_to_env(self._env)
        # self._env.add(self.ax)
        if self._gripper_ready:
            self.gripper.base = self.fkine(self.get_jointstates())
            self.gripper.add_to_env(self._env)
        
    def get_workspace(self):
        """ 
        Get workspace of the robot
        """
        step = np.pi/4
        pointcloud_size = 5*int(
            np.prod(
                np.floor((self.qlim[1, 1:6] - self.qlim[0, 1:6]) / step + 1)
            )
        )
        print(pointcloud_size)
        
        pointcloud = np.zeros((pointcloud_size,3))
        counter = 0
        start_time = time.time()
        print("Start getting point cloud...")
        
        for q0 in np.arange(self.qlim[0,0], self.qlim[1,0] + step, step):
            for q1 in np.range(self.qlim[0,1], self.qlim[1,1] + step, step):
                q = [q0,q1]
                
                ee_pose = self.fkine(q).A
                
                if counter == pointcloud_size:
                    break
                
                pointcloud[counter, :] = ee_pose[0:3, 3]
                counter += 1
                if np.mod(counter / pointcloud_size * 100, 1)==0:
                    end_time = time.time()
                    execution_time = end_time - start_time
                    print (
                        f"After {execution_time} seconds, complete",
                        counter / pointcloud_size * 100,
                        "% of pose",
                    )
                    
        
        # Eliminate points that lie below the table
        pointcloud = pointcloud[pointcloud[:, 2] > 0.1+self.base.A[2, 3], :]
        pc_x = pointcloud[:, 0]
        pc_y = pointcloud[:, 1]
        print('max x: ', np.max(pc_x), 'min x: ', np.min(pc_x))
        print('max y: ', np.max(pc_y), 'min y: ', np.min(pc_y))
        print("Point cloud complete with %d points captured!", len(pointcloud))
        return pointcloud
    
    
    def send_joint_command(self, q):
        """
        Send joint command to robot. Current mode available is joint position mode
        """
        self.q = q
        # self.ax.T = self.fkine(self.q) @ self.gripper_offset
        if self._gripper_ready:
            self.gripper.base = self.fkine(self.get_jointstates())
        self._env.step(0)  
    
    def open_gripper(self):
        self.gripper.open()
        
    def close_gripper(self):
        self.gripper.close()
    
    def test(self):
        """
        Test the class by adding 3d objects into a new Swift window and do a simple movement
        """

        self.q = self._qtest
        self.update_sim()

        q_goal = [np.pi/2, -np.pi/4, 0, 0, 0, 20*np.pi/2]
        qtraj = rtb.jtraj(self.q, q_goal, 50).q

        fig = self.plot(self.q, limits= [-0.1,0.4,-0.3,0.3,-0.1,0.4])
        fig._add_teach_panel(self, self.q)
        for q in qtraj:
            self.q = q  # Robot jointstate
            self._env.step(0.02)
            fig.step(0.01)
        fig.hold()

        self._env.hold()
    
    

    
#----------- CLASS GRIPPER
class Finger(M_DHRobot3D):
    
    def __init__(self, base, links, link3D_names, name = None, qtest = None):
        qtest = [0]
        finger_transform = [
            smb.transl(-0.055,0,0) @ smb.troty(-np.pi/2), # NEED TO FIX
            smb.transl(-0.09,0,0)   # NEED TO FIX
        ]
        
        current_path = os.path.abspath(os.path.dirname(__file__))
        gripper_path = os.path.join(current_path, "Astorino_model")
           
        super().__init__(
            links,
            link3D_names,
            gripper_path,
            name,
            qtest,
            qtest_transforms=finger_transform,
        )
        self.base = base
    
    
class AstorinoGripper:
    
    _qtest = [0]
    
    @property
    def base(self):
        return self._base
    
    def __init__(self, base = sm.SE3(0, 0, 0)):
        links = self._create_DH()
        base_init = sm.SE3(0, 0, 0)
        self._base = base
        
        right_link3D_names = dict(
            link0 = "Astorino_Robot_Gripper_Base",
            link1 = "Astorino_Robot_Gripper_Right"
        )
        
        self._right_finger = Finger(
            base_init,
            links,
            right_link3D_names,
            name= "right_f",
            qtest= self._qtest
        )
        
        self.base_tf_right = sm.SE3.Ry(90,'deg') * sm.SE3(0.007,0,0)  # NEED TO FIX
        self._right_finger.base = (
            self._base.A @ self.base_tf_right.A
        )
        
        #---------------------------------------------

        # Left finger properties

        left_link3D_names = dict(
            link0 = "Astorino_Robot_Gripper_Base",
            link1 = "Astorino_Robot_Gripper_Left"
        )

        self._left_finger = Finger(
            base_init, 
            links, 
            left_link3D_names, 
            name="left_f", 
            qtest=self._qtest
        )

        self.base_tf_left = sm.SE3.Ry(90,'deg') * sm.SE3(0.007,0,0) # NEED TO FIX
        self._left_finger.base = (
            self._base.A @ self.base_tf_left.A
        )

    # -----------------------------------------------------------------------------------#
    def close(self):
        """
        Function to close gripper model
        """
        q_goal = np.array([-0.0005])
        steps = 20
        qtraj_left = rtb.jtraj(self._left_finger.q, q_goal,steps).q
        qtraj_right = rtb.jtraj(self._right_finger.q, -1*q_goal,steps).q
        
        for i in range (steps):
            self._left_finger.q = qtraj_left[i]
            self._right_finger.q = qtraj_right[i]
            self._env.step(0.02)
            
    def open(self):
        """
        Function to open gripper model
        """
        q_goal = np.array([0.02])
        steps = 20
        qtraj_left = rtb.jtraj(self._left_finger.q, q_goal,steps).q
        qtraj_right = rtb.jtraj(self._right_finger.q, -1*q_goal,steps).q
        
        for i in range (steps):
            self._left_finger.q = qtraj_left[i]
            self._right_finger.q = qtraj_right[i]
            self._env.step(0.02) 

    # -----------------------------------------------------------------------------------#
    def _create_DH(self):
        """
        The gripper chosen to use for this mission is Onrobot gripper RG6

            Gripper model is constructed by one base with two fingers
            Base is considered as a statis object attached to robot end-effectorand transform along with ee pose .
            Two fingers are models as two 2-links plannar robots with a constraint for the ee of those plannar always align local z axis of the gripper base
        """
        links = []
        
        link = rtb.PrismaticDH(alpha= 0, offset= 0, qlim= [-24.19, 24.19])
        links.append(link)
        
        return links
    
    # -----------------------------------------------------------------------------------#
    def add_to_env(self, env):
        """
        Function to add 2 fingers model to environment
        """

        # add 2 fingers to the environment

        self._env = env
        self._right_finger.add_to_env(env)
        self._left_finger.add_to_env(env)
        
    # -----------------------------------------------------------------------------------#
    def set_base(self, newbase):
        """
        Function to update gripper base
        """

        # assign new base for the gripper to be attached with robot ee given

        self._right_finger.base = newbase.A @ self.base_tf_right.A
        self._left_finger.base = newbase.A @ self.base_tf_left.A


        # give empty q so that gripper model is updated on swift

        self._right_finger.q = self._right_finger.q
        self._left_finger.q = self._left_finger.q

    # -----------------------------------------------------------------------------------#
    def __setattr__(self, name, value):
        """
        Overload `=` operator so the object can update its 3D model whenever a new base is assigned
        """

        if name == "base" and hasattr(self, "base"):
            self.set_base(value)  # Update the 3D model before setting the attribute
        else:
            super().__setattr__(
                name, value
            )  # Call the base class method to set the attribute
    
# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":

    # generate environment
    env = Swift()
    env.launch(realtime=True)

    # generate robot
    r = Astorino(env, gripper_ready= True)
    
    time.sleep(1)
    q_goal = [0, 0, 0, 0, 0, 0]
    qtraj = rtb.jtraj(r.q, q_goal, 200).q
    for q in qtraj:
        r.send_joint_command(q)
        time.sleep(0.02)
    r.open_gripper()
    r.close_gripper()  
    env.hold()

