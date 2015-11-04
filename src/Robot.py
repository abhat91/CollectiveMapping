import numpy as np

class Robot:
    perceptsize=3
    #TODO: Change this based on the size of the world
    perceptmap = np.zeros(shape=(1000, 1000), dtype=int)
    #This position has to be updated on every move
    xmapposition=250
    ymapposition=250

    def expandperceptmap(self, perceptMatrix):
        """Given the percept matrix, the robot adds the percept to the map of the robot"""
        mapstartX=self.xmapposition-(self.perceptsize)/2
        mapstartY=self.ymapposition-(self.perceptsize)/2
        #Adds the percepts to the robot's map of the world
        for i in range(self.perceptsize):
            for j in range(self.perceptsize):
                self.perceptmap[mapstartX+i][mapstartY+j]=perceptMatrix[i][j]
