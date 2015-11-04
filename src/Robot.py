import numpy as np

class Robot:
    perceptradius=3
    #TODO: Change this based on the size of the world
    perceptmap = np.zeros(shape=(1000, 1000), dtype=int)
    #This position has to be updated on every move
    xmapposition=500
    ymapposition=500

    #This value has to be updated with the minimum value of x and y reached by the robot
    minxposition=499
    minyposition=499

    #This value has to be updated with the minimum value of x and y reached by the robot
    maxxposition=501
    maxyposition=501

    def expandperceptmap(self, perceptMatrix):
        """Given the percept matrix, the robot adds the percept to the map of the robot"""
        mapstartX=self.xmapposition-(self.perceptsize)/2
        mapstartY=self.ymapposition-(self.perceptsize)/2
        #Adds the percepts to the robot's map of the world
        for i in range(self.perceptsize):
            for j in range(self.perceptsize):
                self.perceptmap[mapstartX+i][mapstartY+j]=perceptMatrix[i][j]

    #Given 2 robots in proximity, the map of the other robot is taken and stitched to the current map to make a larger map of the environment
    def stitchmaps(self, relativePositionOfOtherRobot, otherRobot):
        """Given 2 robots in proximity, the map of the other robot is taken and stitched to the current map to make a larger map of the environment"""
        rpositionOfOtherRobotX = otherRobot.xmapposition-otherRobot.minxposition
        rpositionOfOtherRobotY = otherRobot.ymapposition-otherRobot.minyposition
        robotmap=otherRobot.perceptmap[otherRobot.minxposition:otherRobot.maxxposition, otherRobot.minyposition:otherRobot.maxyposition]
        shapeofworld=np.shape(robotmap)
        positionxofotherrobot=self.xmapposition-self.perceptradius+relativePositionOfOtherRobot[0]
        positionyofotherrobot=self.ymapposition-self.perceptradius+relativePositionOfOtherRobot[1]
        startmapx=positionxofotherrobot-rpositionOfOtherRobotX
        startmapy=positionyofotherrobot-rpositionOfOtherRobotY
        self.perceptmap[startmapx:startmapx+shapeofworld[0], startmapy:startmapy+shapeofworld[1]]=robotmap

        #Update the min and max positions covered by the map
        if self.minxposition>startmapx:
            self.minxposition=startmapx
        if self.minyposition>startmapy:
            self.minyposition=startmapy

        if self.maxxposition<startmapy+shapeofworld[0]:
            self.maxxposition=startmapy+shapeofworld[0]
        if self.maxyposition>startmapy+shapeofworld[1]:
            self.maxyposition=startmapy+shapeofworld[1]
