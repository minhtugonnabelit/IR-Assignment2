import numpy as np
import matplotlib.pyplot as plt

def make_ellipsoid(ellipsoid_info, center, color=None, u=None, v=None, ax=None, is_plot = True):
        """
        Simple custom function to create an ellipsoid.
        
        :param ellipsoid_info: 1x3 array-like (ellipsoid radii) or 3x3 array (ellipsoid matrix)
        :param center: center of the ellipsoid
        :param color: color for ellipsoid
        :param u: list of azimuthal angle in spherical coordinates (optional)
        :param v: list of polar angle in spherical coordinates (optional)
        :param ax: axis to plot on (optional)
        :return surface object & tuple of mesh data (X, Y, Z)
        """
        if np.shape(ellipsoid_info) == (1, 3) or np.shape(ellipsoid_info) == (3,):
            lengths = ellipsoid_info
            eigenvectors = np.eye(3)
        elif np.shape(ellipsoid_info) == (3, 3):
            eigenvalues, eigenvectors = np.linalg.eig(ellipsoid_info)
            lengths = np.sqrt(np.abs(eigenvalues))
        else:
            raise ValueError('Invalid input ellipsoid info!')

        # Ellipsoid surface
        if u is None:
            u = np.linspace(0, 2 * np.pi, 40)
        if v is None:
            v = np.linspace(0, np.pi, 20)

        # Generate surface points of the ellipsoid
        X = []
        Y = []
        Z = []
        for phi in u:
            for theta in v:
                point = np.array([
                    lengths[0] * np.cos(phi) * np.sin(theta),
                    lengths[1] * np.sin(phi) * np.sin(theta),
                    lengths[2] * np.cos(theta)
                ])
                rotated_point = np.dot(eigenvectors, point)
                X.append(rotated_point[0] + center[0])
                Y.append(rotated_point[1] + center[1])
                Z.append(rotated_point[2] + center[2])

        X = np.array(X).reshape(len(u), len(v))
        Y = np.array(Y).reshape(len(u), len(v))
        Z = np.array(Z).reshape(len(u), len(v))

        if is_plot:
            # Plot ellipsoid
            if ax is None:
                ax = plt.figure().add_subplot(projection='3d')

            if color is None:
                return ax.plot_surface(X, Y, Z, cmap='inferno', alpha=0.5), (X,Y,Z)

            return ax.plot_surface(X, Y, Z, color=color, alpha=0.5), (X,Y,Z)

        else:
            return (X,Y,Z)
        
def get_ellipsoid(self):

    """ Get ellipsoid of the robot """

    ellipsoids = []
    for i in range(len(self.links)):
        
        # special define for major and minor axis for ellipsoid
        minor_axis = 0.03 if self.a[i] == 0 else copy.deepcopy(self.a[i]) / 2 + 0.02
        major_axis = copy.deepcopy(self.d[i]) / 2

        # define major and minor axis of ellipsoid
        ellipsoid = np.asarray([minor_axis, major_axis, minor_axis])

        ellipsoids.append(ellipsoid)

    if self.n == 7: # 7dof robot
        ellipsoids[6][2] = ellipsoids[6][1]
        ellipsoids[6][1] = ellipsoids[6][0]
    else: # 6dof robot
        ellipsoids[5][2] = ellipsoids[5][1]
        ellipsoids[5][1] = ellipsoids[5][0]

    return ellipsoids

def get_link_poses(self, q=None):
    """
    :param q robot joint angles
    :param robot -  seriallink robot model
    :param transforms - list of transforms
    """
    if q is None:
        return self.fkine_all()
    return self.fkine_all(q)

def get_ellipsoid_meshlist(self):
    """
    Get the ellipsoid mesh points corresponded to each link stick to their local frame"""
    meshlist = []

    for ellipsoid in self._ellipsoids:
        meshlist.append(make_ellipsoid(ellipsoid, np.zeros(3), is_plot = False))

    return meshlist   

def transform_ellipsoid(self, links_center):

    # get the ellipsoid mesh points corresponded to each link and transform to the world frames
    ellip_transforms = []
    for i in range(len(self._ellipsoids_meshlist)):
        
        ellip = []
        mesh = np.array(self._ellipsoids_meshlist[i])

        # Reshaping the mesh points to be a two-dimensional array and adding a row of ones for homogeneous coordinates
        homogeneous_mesh_points = np.vstack((mesh.reshape(3, -1), np.ones((1, mesh.shape[1] * mesh.shape[2]))))
        
        # Applying the transformation to the mesh points with corresponding link centers for mesh points relative to world frame
        transformed_points = links_center[i] @ homogeneous_mesh_points

        # Removing the last row to return to (x, y, z) coordinates and reshaping to the original structure
        ellip = transformed_points[:-1, :]
        ellip_transforms.append(ellip)
    
    return ellip_transforms


# collision function:
def collision_check(self, q, object : geometry.Mesh):
    """
    """
    # get the transforms of all links
    ee_pose = self.fkine(q)
    ee_sphere = geometry.Cylinder(0.03, self.d[self.n-1]/2,  pose = ee_pose)
    d, p1, p2 = ee_sphere.closest_point(object, 5)
    return d, p1, p2

def object_collision_check(self, q, vecteces, faces, normals):
    """
    Standard version using line-plane intersection
    """
    links_tf = self.get_link_poses(q)


    return False

def is_self_collision(self, q):
    """
    Self collision check using ellipsoid.
    Principle is from extracting the ellipsoid mesh points corresponded to each link, then transform its to each link ceener to check if there is any intersection
    """

    # get the transforms of all links
    links_tf = self.get_link_poses(q)

    # get the center of each link
    links_center = []
    for i in range(len(links_tf)-1):
        links_center.append((links_tf[i+1] + links_tf[i]) / 2)

    # get the ellipsoid mesh points corresponded to each link and transform to the world frames
    ellip_transforms = self.transform_ellipsoid(links_center)

    # iteration through each link
    for i, cur_center in enumerate(links_center):

        # iteration through each link but avoid the currentt link and neighbor links
        for j in range(len(links_center)):

            if abs(j-i) <= 1:
                continue
            
            # iteration through each ellipsoid mesh points
            for ellip in ellip_transforms[j]:
                
                # transform the extracted ellipsoid mesh points to the local frame of the current link
                transformed_ellip = np.linalg.inv(cur_center) @ ellip

                # check if the transformed ellipsoid mesh points is inside the current link
                if transformed_ellip[0]**2 / self._ellipsoids[i][0]**2 + transformed_ellip[1]**2 / self._ellipsoids[i][1]**2 + transformed_ellip[2]**2 / self._ellipsoids[i][2]**2 <= 1:
                    return True

    return False





def grounded_check(self, q, ground_height = None):
    """
    """
    # if ground_height is not defined, use the base height as the ground height
    if ground_height is None:
        ground_height = self.base.A[2, 3]

    # get the transforms of all links
    link_transforms = self.get_link_poses(q)

    # check if all links are above the ground defined
    for j, link in enumerate(link_transforms):
        if j <= 1:
            continue
        elif link[2, 3] < ground_height +  0.02:
            return True
    return False 