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

class Astorino(M_DHRobot3D):
    
    _NEUTRAL = [0, -np.pi/2, 0, 0, 0, 0]
    _script_directory = os.path.dirname(os.path.abspath(__file__))
    # def __init__(self, links, link3D_names, link3d_dir, name=None, qtest=None, qtest_transforms=None):
    #     super().__init__(links, link3D_names, link3d_dir, name, qtest, qtest_transforms)
    
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
            link0 = 'Astorino base',
            link1 = 'Astorino Robot Joint 1',
            link2 = 'Astorino Robot Joint 2',
            link3 = 'Astorino Robot Joint 3',
            link4 = 'Astorino Robot Joint 4',
            link5 = 'Astorino Robot Joint 5',
            link6 = 'Astorino Robot Joint 6'
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
        
    
    def _add_model(self, file_path, placement, color=None):
        """ 
        Add model to simulation environment
        """
        model_full_path = os.path.join(self._script_directory, file_path)
        model_placement = self.base.A @ placement
        if color is None:
            model = geometry.Mesh(
                model_full_path, pose = model_placement)
        else:
            model = geometry.Mesh(
                model_full_path, pose = model_placement, color = color)
            
        self._env.add(model, collision_alpha=1)
        return model
        
    def update_sim(self):
        """ 
        Update simulation
        """
        self.add_to_env(self._env)
        
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
class Gripper_Astorino():
    def __init__(self):
        self.Gripper_Astorino_Right = GripperRIGHT_Astorino()
        self.Gripper_Astorino_Left = GripperRIGHT_Astorino()
    
    
# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":

    # generate environment
    env = Swift()
    env.launch(realtime=True)

    # generate robot
    r = Astorino(env)
    r.test()
