import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D

class RectangularPrism:
    def __init__(self, width=1, breadth=1, height=1, color=[0, 0, 1], center=[0, 0, 0]):
        self.width = width
        self.breadth = breadth
        self.height = height
        self.center = center
        self.color = color
        self.options = []
        
        self._vertices = self._calculate_vertices()
        self._faces = self._calculate_faces()
        self._normals = self._calculate_normals()
        self.obj = Poly3DCollection(self._vertices[self._faces], linewidths=0.5, edgecolors='k')
        # self.obj.set_facecolor(color)

        # self._plot_prism()
    
    def _calculate_vertices(self):
        vertices = np.zeros((8, 3))
        vertices[1] = [self.width, 0, 0]
        vertices[2] = [self.width, self.breadth, 0]
        vertices[3] = [0, self.breadth, 0]
        vertices[4] = [0, 0, self.height]
        vertices[5] = [self.width, 0, self.height]
        vertices[6] = [self.width, self.breadth, self.height]
        vertices[7] = [0, self.breadth, self.height]
        
        # Shift vertices based on the center point
        vertices -= np.array([self.width/2, self.breadth/2, self.height/2])  # Subtract half of width, breadth, and height
    
        # Translate the vertices to the specified center
        vertices += np.array(self.center)
        
        return vertices
    
    def _calculate_faces(self):
        faces = [
            [0, 1, 2, 3],  # z == 0
            [0, 4, 7, 3],  # x == 0
            [0, 1, 5, 4],  # y == 0
            [2, 6, 5, 1],  # x == width
            [3, 7, 6, 2],  # y == breadth
            [4, 5, 6, 7]   # z == height
        ]
        return faces
    
    def _calculate_normals(self):
        normals = []
        for face in self._faces:
            v1 = self._vertices[face[1]] - self._vertices[face[0]]
            v2 = self._vertices[face[2]] - self._vertices[face[1]]
            normal = np.cross(v1, v2)
            normals.append(normal)
        return normals
    
    # def _plot_prism(self):
    #     existing_axes = plt.gcf().axes
    #     if len(existing_axes) == 0: # If no current axes, then create one
    #         ax = plt.gcf().add_subplot(projection='3d')
    #     else: # If there is a current axes, check and plot on that one
    #         ax = plt.gca()
    #         if not isinstance(ax, Axes3D):
    #             ax = plt.gcf().add_subplot(projection='3d')
        
    #     ax.add_collection(self.obj)
    
    def get_data(self):
        return self._vertices, self._faces, self._normals
    
# ---------------------------------------------------------------------------------------#
if __name__ == '__main__':
    plt.figure().add_subplot(projection= '3d')
    r = RectangularPrism(0.5,0.5,0.5)
    plt.show()