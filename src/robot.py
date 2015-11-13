import numpy as np
import utils
direction = {
0 : utils.MOVES.NORTH,
1 : utils.MOVES.NORTHEAST,
2 : utils.MOVES.EAST,
3 : utils.MOVES.SOUTHEAST,
4 : utils.MOVES.SOUTH,
5 : utils.MOVES.SOUTHWEST,
6 : utils.MOVES.WEST,
7 : utils.MOVES.NORTHWEST


}
class Robot:
    perceptradius=1
    #TODO: Change this based on the size of the world
    perceptmap = np.zeros(shape=(20, 20), dtype=int)
    #This position has to be updated on every move
    xmapposition=10
    ymapposition=10

    #This value has to be updated with the minimum value of x and y reached by the robot
    minxposition=9
    minyposition=9

    #This value has to be updated with the minimum value of x and y reached by the robot
    maxxposition=11
    maxyposition=11
    world = None
    def __init__(self,world):
        l = len(world)
        self.perceptmap = np.zeros(shape=(2*l,2*l),dtype=int)
        self.xmapposition=l
        self.ymapposition=l

        #This value has to be updated with the minimum value of x and y reached by the robot
        self.minxposition=l - self.perceptradius
        self.minyposition= l- self.perceptradius

        #This value has to be updated with the minimum value of x and y reached by the robot
        self.maxxposition= l + self.perceptradius
        self.maxyposition= l + self.perceptradius
        self.world = world

    def expandperceptmap(self, perceptMatrix):
        """Given the percept matrix, the robot adds the percept to the map of the robot"""
        mapstartX=self.xmapposition-(self.perceptradius)
        mapstartY=self.ymapposition-(self.perceptradius)
        #Adds the percepts to the robot's map of the world
        for i in range((2*self.perceptradius)+1):
            for j in range((2*self.perceptradius)+1):
                self.perceptmap[mapstartX+i][mapstartY+j]=perceptMatrix[i][j]

    #Given 2 robots in proximity, the map of the other robot is taken and stitched to the current map to make a larger map of the environment
    def stitchmaps(self, relativePositionOfOtherRobot, otherRobot):
        """Given 2 robots in proximity, the map of the other robot is taken and stitched to the current map to make a larger map of the environment"""
        rpositionOfOtherRobotX = otherRobot.xmapposition-otherRobot.minxposition
        rpositionOfOtherRobotY = otherRobot.ymapposition-otherRobot.minyposition
        robotmap=otherRobot.perceptmap[otherRobot.minxposition:otherRobot.maxxposition+1, otherRobot.minyposition:otherRobot.maxyposition+1]
        robotmap[robotmap==utils.MAPREP.SELF]=utils.MAPREP.EMPTY
        shapeofworld=np.shape(robotmap)
        positionxofotherrobot=self.xmapposition-self.perceptradius+relativePositionOfOtherRobot[0]
        positionyofotherrobot=self.ymapposition-self.perceptradius+relativePositionOfOtherRobot[1]
        startmapx=positionxofotherrobot-rpositionOfOtherRobotX
        startmapy=positionyofotherrobot-rpositionOfOtherRobotY
        newsubmap=np.maximum.reduce([self.perceptmap[startmapx:startmapx+shapeofworld[0], startmapy:startmapy+shapeofworld[1]], robotmap])
        self.perceptmap[startmapx:startmapx+shapeofworld[0], startmapy:startmapy+shapeofworld[1]]=newsubmap

        #Update the min and max positions covered by the map
        if self.minxposition>startmapx:
            self.minxposition=startmapx
        if self.minyposition>startmapy:
            self.minyposition=startmapy

        if self.maxxposition<startmapy+shapeofworld[0]:
            self.maxxposition=startmapy+shapeofworld[0]
        if self.maxyposition>startmapy+shapeofworld[1]:
            self.maxyposition=startmapy+shapeofworld[1]
    def move(self,dir):
        if self.world.robotMove(self,dir):
        #move successful, Update percept map
            robots, percept = world.getsubmap(self)
            self.expandperceptmap(percept)
            return robots
        return []



