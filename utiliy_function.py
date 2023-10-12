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