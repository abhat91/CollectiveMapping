import numpy as np
import utils
import random
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

    world = None
    def __init__(self,world,maplen):
        l = maplen
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

    def updateminimumpositions(self):
        if self.xmapposition - self.perceptradius < self.minxposition:
            self.minxposition=self.xmapposition-self.perceptradius
        if self.ymapposition - self.perceptradius < self.minyposition:
            self.minyposition=self.ymapposition-self.perceptradius

    def updatemaximumpositions(self):
        if self.xmapposition + self.perceptradius > self.maxxposition:
            self.maxxposition=self.xmapposition+self.perceptradius
        if self.ymapposition + self.perceptradius > self.maxyposition:
            self.maxyposition=self.ymapposition+self.perceptradius


    #Given 2 robots in proximity, the map of the other robot is taken and stitched to the current map to make a larger map of the environment
    def stitchmaps(self, relativePositionOfOtherRobot, otherRobot):
        """Given 2 robots in proximity, the map of the other robot is taken and stitched to the current map to make a larger map of the environment"""
        rpositionOfOtherRobotX = otherRobot.xmapposition-otherRobot.minxposition
        rpositionOfOtherRobotY = otherRobot.ymapposition-otherRobot.minyposition
        robotmap=otherRobot.perceptmap[otherRobot.minxposition:otherRobot.maxxposition+1, otherRobot.minyposition:otherRobot.maxyposition+1]
        robotmap[robotmap == utils.MAPREP.SELF]=utils.MAPREP.EMPTY
        robotmap[robotmap == utils.MAPREP.PEER]=utils.MAPREP.EMPTY
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

        if self.maxxposition<startmapx+shapeofworld[0]:
            self.maxxposition=startmapx+shapeofworld[0]-1
        if self.maxyposition>startmapy+shapeofworld[1]:
            self.maxyposition=startmapy+shapeofworld[1]-1

    def move(self,dir):
        if self.world.robotMove(self,dir):
        #move successful, Update percept map
            self.xmapposition+=dir[0]
            self.ymapposition+=dir[1]
            self.updatemaximumpositions()
            self.updateminimumpositions()
            robots, percept = self.world.getsubmap(self)
            self.expandperceptmap(percept)
            self.stoppingcriterion()
            return robots
        return []

    def randomMove(self):
        robotslist = self.move(direction[int(random.random()*8)])
        if len(robotslist)>0:
            for relativepos,robot in robotslist:
                self.stitchmaps(relativepos,robot)

    def stoppingcriterion(self):
        maptolookup=self.perceptmap[self.minxposition: self.maxxposition+1, self.minyposition:self.maxyposition+1]
