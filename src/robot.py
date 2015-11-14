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
        """Initialization method of the robot class. Updates the x and y positions. Also updates the minimum and maximum x and y
        positions"""
        l = maplen
        self.perceptmap = np.zeros( shape= (2*l, 2*l), dtype=int)
        self.xmapposition = l
        self.ymapposition = l

        #This value has to be updated with the minimum value of x and y reached by the robot
        self.minxposition = l - self.perceptradius
        self.minyposition = l- self.perceptradius

        #This value has to be updated with the minimum value of x and y reached by the robot
        self.maxxposition = l + self.perceptradius
        self.maxyposition = l + self.perceptradius
        self.world = world

    def expandperceptmap(self, perceptMatrix):
        """Given the percept matrix, the robot adds the percept to the map of the robot"""
        mapstartX = self.xmapposition - (self.perceptradius)
        mapstartY = self.ymapposition- ( self.perceptradius)
        #Adds the percepts to the robot's map of the world
        for i in range( (2*self.perceptradius) + 1 ):
            for j in range( (2*self.perceptradius) + 1 ):
                self.perceptmap[mapstartX+i][mapstartY+j] = perceptMatrix[i][j]

    def updateminimumpositions(self):
        """Updates the minimum positions of each robot"""
        if self.xmapposition - self.perceptradius < self.minxposition:
            self.minxposition = self.xmapposition - self.perceptradius
        if self.ymapposition - self.perceptradius < self.minyposition:
            self.minyposition = self.ymapposition - self.perceptradius

    def updatemaximumpositions(self):
        """Updates the maximum positions of each robot"""
        if self.xmapposition + self.perceptradius > self.maxxposition:
            self.maxxposition = self.xmapposition + self.perceptradius
        if self.ymapposition + self.perceptradius > self.maxyposition:
            self.maxyposition = self.ymapposition + self.perceptradius


    #Given 2 robots in proximity, the map of the other robot is taken and stitched to the current map to make a larger map of the environment
    def stitchmaps(self, relativePositionOfOtherRobot, otherRobot):
        """Given 2 robots in proximity, the map of the other robot is taken and stitched to the current map to make a larger map of the environment"""
        rpositionOfOtherRobotX = otherRobot.xmapposition - otherRobot.minxposition
        rpositionOfOtherRobotY = otherRobot.ymapposition - otherRobot.minyposition
        robotmap = otherRobot.perceptmap[ otherRobot.minxposition:otherRobot.maxxposition+1, otherRobot.minyposition:otherRobot.maxyposition+1 ]
        robotmap[robotmap == utils.MAPREP.SELF] = utils.MAPREP.EMPTY
        robotmap[robotmap == utils.MAPREP.PEER] = utils.MAPREP.EMPTY
        shapeofworld = np.shape(robotmap)
        positionxofotherrobot = self.xmapposition-self.perceptradius+relativePositionOfOtherRobot[0]
        positionyofotherrobot = self.ymapposition-self.perceptradius+relativePositionOfOtherRobot[1]
        startmapx = positionxofotherrobot-rpositionOfOtherRobotX
        startmapy = positionyofotherrobot-rpositionOfOtherRobotY
        newsubmap = np.maximum.reduce([self.perceptmap[startmapx:startmapx+shapeofworld[0], startmapy:startmapy+shapeofworld[1]], robotmap])
        self.perceptmap[startmapx:startmapx+shapeofworld[0], startmapy:startmapy+shapeofworld[1]] = newsubmap

        #Update the min and max positions covered by the map
        if self.minxposition>startmapx:
            self.minxposition = startmapx
        if self.minyposition > startmapy:
            self.minyposition = startmapy

        if self.maxxposition < startmapx+shapeofworld[0]:
            self.maxxposition = startmapx+shapeofworld[0] - 1
        if self.maxyposition > startmapy+shapeofworld[1]:
            self.maxyposition = startmapy+shapeofworld[1] - 1

    def move(self,dir):
        """Moves the robot by one position and updates the map"""
        if self.world.robotMove(self, dir):
        #move successful, Update percept map
            self.xmapposition += dir[0]
            self.ymapposition += dir[1]
            self.updatemaximumpositions()
            self.updateminimumpositions()
            robots, percept = self.world.getsubmap(self)
            self.expandperceptmap(percept)
            self.stoppingcriterion()
            return robots
        return []

    def randomMove(self):
        """Random movement algorithm"""
        robotslist = self.move(direction[int(random.random()*8)])
        if len(robotslist) > 0:
            for relativepos,robot in robotslist:
                self.stitchmaps(relativepos,robot)

    def adimove(self):
        #Move randomly first a few times
        dim=len(self.perceptmap)/4
        northwest=self.perceptmap[self.xmapposition-dim:self.xmapposition, self.ymapposition-dim:self.ymapposition]
        northeast=self.perceptmap[self.xmapposition:self.xmapposition+dim, self.ymapposition-dim:self.ymapposition]
        southwest=self.perceptmap[self.xmapposition-dim:self.xmapposition, self.ymapposition:self.ymapposition+dim]
        southeast=self.perceptmap[self.xmapposition:self.xmapposition+dim, self.ymapposition:self.ymapposition+dim]
        val=[np.count_nonzero(northwest), np.count_nonzero(northeast), np.count_nonzero(southwest), np.count_nonzero(southeast)]
        if val.index(min(val))==0:
            x=[utils.MOVES.NORTHWEST, utils.MOVES.NORTH, utils.MOVES.WEST]
            robotslist = self.move(random.choice(x))
        elif val.index(min(val))==1:
            x=[utils.MOVES.NORTHEAST, utils.MOVES.NORTH, utils.MOVES.EAST]
            robotslist = self.move(random.choice(x))
        elif val.index(min(val))==2:
            x=[utils.MOVES.SOUTHWEST, utils.MOVES.SOUTH, utils.MOVES.WEST]
            robotslist = self.move(random.choice(x))
        elif val.index(min(val))==3:
            x=[utils.MOVES.SOUTHEAST, utils.MOVES.SOUTH, utils.MOVES.EAST]
            robotslist = self.move(random.choice(x))
        if len(robotslist) > 0:
            for relativepos,robot in robotslist:
                self.stitchmaps(relativepos,robot)


    def stoppingcriterion(self):
        maptolookup = self.perceptmap[ self.minxposition: self.maxxposition+1, self.minyposition:self.maxyposition+1 ]
        currentxposition = self.xmapposition
        currentyposition = self.ymapposition
