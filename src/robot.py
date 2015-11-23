import numpy as np
import utils
import random
import copy
import sys
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
previousMoveProbs = {
0 : [20,0.8,0.4,0.25,0.1,0.25,0.4,0.8],
1 : [0.8,20,0.8,0.4,0.25,0.1,0.25,0.4],
2 : [0.4,0.8,20,0.8,0.4,0.25,0.1,0.25],
3 : [0.25,0.4,0.8,20,0.8,0.4,0.25,0.1],
4 : [0.1,0.25,0.4,0.8,20,0.8,0.4,0.25],
5 : [0.25,0.1,0.25,0.4,0.8,20,0.8,0.4],
6 : [0.4,0.25,0.1,0.25,0.4,0.8,20,0.8],
7 : [0.8,0.4,0.25,0.1,0.25,0.4,0.8,20],
}
class Robot:
    perceptradius=1
    previousMove = None
    world = None
    def __init__(self,world,maplen):
        """Initialization method of the robot class. Updates the x and y positions. Also updates the minimum and maximum x and y
        positions"""
        l = maplen
        self.perceptmap = np.zeros( shape= (2*l, 2*l), dtype=int)
        self.currentPercept = np.zeros((self.perceptradius*2+1,self.perceptradius*2+1))
        self.xmapposition = l
        self.ymapposition = l

        #This value has to be updated with the minimum value of x and y reached by the robot
        self.minxposition = l - self.perceptradius
        self.minyposition = l- self.perceptradius

        #This value has to be updated with the minimum value of x and y reached by the robot
        self.maxxposition = l + self.perceptradius
        self.maxyposition = l + self.perceptradius
        self.world = world
        self.previousMove = None
    

    def expandperceptmap(self):
        """Given the percept matrix, the robot adds the percept to the map of the robot"""
        mapstartX = self.xmapposition - (self.perceptradius)
        mapstartY = self.ymapposition- ( self.perceptradius)
        #Adds the percepts to the robot's map of the world
        #print self.currentPercept
        for i in range( (2*self.perceptradius) + 1 ):
            for j in range( (2*self.perceptradius) + 1 ):
                self.perceptmap[mapstartX+i,mapstartY+j] = self.currentPercept[i,j]
        self.perceptmap[self.xmapposition,self.ymapposition] = utils.MAPREP.SELF
        self.perceptmap[self.perceptmap == utils.MAPREP.PEER] = utils.MAPREP.EMPTY

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
        robotmap = copy.deepcopy(otherRobot.perceptmap[ otherRobot.minxposition:otherRobot.maxxposition+1, otherRobot.minyposition:otherRobot.maxyposition+1 ])
        robotmap[robotmap == utils.MAPREP.SELF] = utils.MAPREP.EMPTY
        robotmap[robotmap == utils.MAPREP.PEER] = utils.MAPREP.EMPTY
        shapeofworld = np.shape(robotmap)
        positionxofotherrobot = self.xmapposition-self.perceptradius+relativePositionOfOtherRobot[0]
        positionyofotherrobot = self.ymapposition-self.perceptradius+relativePositionOfOtherRobot[1]
        startmapx = positionxofotherrobot-rpositionOfOtherRobotX
        startmapy = positionyofotherrobot-rpositionOfOtherRobotY
        newsubmap = np.maximum.reduce([self.perceptmap[startmapx:startmapx+shapeofworld[0], startmapy:startmapy+shapeofworld[1]], robotmap])
        self.perceptmap[startmapx:startmapx+shapeofworld[0], startmapy:startmapy+shapeofworld[1]] = newsubmap
        self.perceptmap[self.xmapposition, self.ymapposition]=utils.MAPREP.SELF
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
            self.perceptmap[self.xmapposition,self.ymapposition]=utils.MAPREP.EMPTY
        #move successful, Update percept map
            self.xmapposition += dir[0]
            self.ymapposition += dir[1]
            self.updatemaximumpositions()
            self.updateminimumpositions()
            self.perceptmap[self.xmapposition,self.ymapposition]=utils.MAPREP.SELF
    def bayesMove(self):
        '''@author:renatogg - Moves to a direction with probability given by previous movement'''
        robots, self.currentPercept = self.world.getsubmap(self)
        self.expandperceptmap()
        if self.stoppingcriterion()==True:
            return 'Explored'
        if len(robots) > 0:
            for relativepos,robot in robots:
                self.stitchmaps(relativepos,robot)
        #robot.stitchmaps((-relativepos[0],-relativepos[1]),self)
        if self.previousMove == None:
            return self.randomMove()
        possiblemoves = self.getPossibleMoves()
        if len(possiblemoves) > 0:
            totalweight = sum([previousMoveProbs[self.previousMove][i] for i in possiblemoves])
            decision = random.random()*totalweight
            j = 0
            cumulative = previousMoveProbs[self.previousMove][possiblemoves[j]]
            while cumulative < decision:
                j += 1
                cumulative+=previousMoveProbs[self.previousMove][possiblemoves[j]]
            self.previousMove =possiblemoves[j]
            self.move(direction[self.previousMove])
    def randomMove(self):
        """Random movement algorithm"""
        robots, self.currentPercept = self.world.getsubmap(self)
        self.expandperceptmap()
        if self.stoppingcriterion()==True:
            return 'Explored'
        if len(robots) > 0:
            for relativepos,robot in robots:
                self.stitchmaps(relativepos,robot)
        #robot.stitchmaps((-relativepos[0],-relativepos[1]),self)
        possiblemoves= self.getPossibleMoves()
        if len(possiblemoves)>0:
            #print possiblemoves
            self.previousMove = possiblemoves[int(random.random()*len(possiblemoves))]
            self.move(direction[self.previousMove])

    
    def getPossibleMoves(self):
        possiblemoves = []
        for i in direction.keys():
            dir = direction[i]
            dx = dir[0]
            dy = dir[1]
            if self.currentPercept[self.perceptradius+dx,self.perceptradius+dy] == utils.MAPREP.EMPTY:#direction it wants to move is empty
                if abs(dx) == 1 and abs(dy) == 1:#if direction is diagonal
                    if self.currentPercept[self.perceptradius+dx,self.perceptradius] == utils.MAPREP.EMPTY and self.currentPercept[self.perceptradius,self.perceptradius+dy] == utils.MAPREP.EMPTY:#if diagonal is clear
                        possiblemoves.append(i)
                else:#if it's not a diagonal
                    possiblemoves.append(i)
        return possiblemoves

    def gradientmove(self):
        #Move randomly first a few times
        sizeofsubblockx=(self.maxxposition-self.minxposition)/2
        sizeofsubblocky=(self.maxyposition-self.minyposition)/2
        northwest=self.perceptmap[self.minxposition:self.minxposition+sizeofsubblockx, self.minyposition:self.minyposition+sizeofsubblocky]
        northeast=self.perceptmap[self.minxposition:self.minxposition+sizeofsubblockx, self.minyposition+sizeofsubblocky:self.maxyposition]
        southwest=self.perceptmap[self.minxposition+sizeofsubblockx:self.maxxposition, self.minyposition:self.minyposition+sizeofsubblocky]
        southeast=self.perceptmap[self.minxposition+sizeofsubblockx:self.maxxposition, self.minyposition+sizeofsubblocky: self.maxyposition]
        val=[(np.size(northwest)-np.count_nonzero(northwest))/np.size(northwest), (np.size(northeast)-np.count_nonzero(northeast))/np.size(northeast), (np.size(northeast)-np.count_nonzero(southwest))/np.size(southwest), (np.size(northeast)-np.count_nonzero(southeast))/np.size(southeast)]
        x=0
        robots, self.currentPercept = self.world.getsubmap(self)         
        if len(robots) > 0:
            for relativepos,robot in robots:
                self.stitchmaps(relativepos,robot)
        while True:
            if val.index(max(val))==0:
                if random.random()<0.3:
                    x=[utils.MOVES.NORTHWEST, utils.MOVES.NORTH, utils.MOVES.WEST]
                else:
                    x=[utils.MOVES.NORTHEAST, utils.MOVES.EAST, utils.MOVES.SOUTHEAST, utils.MOVES.SOUTH, utils.MOVES.SOUTHWEST]
            elif val.index(max(val))==1:
                if random.random()<0.3:
                    x=[utils.MOVES.NORTHEAST, utils.MOVES.NORTH, utils.MOVES.EAST]
                else:
                    x=[utils.MOVES.NORTHWEST, utils.MOVES.WEST, utils.MOVES.SOUTHEAST, utils.MOVES.SOUTH, utils.MOVES.SOUTHWEST]
            elif val.index(max(val))==2:
                if random.random()<0.3:
                    x=[utils.MOVES.SOUTHWEST, utils.MOVES.SOUTH, utils.MOVES.WEST]
                else:
                    x=[utils.MOVES.NORTH, utils.MOVES.NORTHWEST, utils.MOVES.NORTHEAST, utils.MOVES.EAST, utils.MOVES.SOUTHEAST]             
            elif val.index(max(val))==3:
                if random.random()<0.3:
                    x=[utils.MOVES.SOUTHEAST, utils.MOVES.SOUTH, utils.MOVES.EAST]
                else:
                    x=[utils.MOVES.SOUTHWEST, utils.MOVES.WEST, utils.MOVES.NORTHWEST, utils.MOVES.NORTH, utils.MOVES.NORTHEAST]
            movechoice=random.choice(x)
            if self.perceptmap[self.xmapposition+movechoice[0], self.ymapposition+movechoice[1]]!=utils.MAPREP.BLOCKED:
                if self.previousMove==movechoice:
                        self.move(movechoice)
                        self.previousMove=movechoice
                        break
                else:
                    if random.random()<0.4:
                        self.move(movechoice)
                        self.previousMove=movechoice
                        break
        robots, self.currentPercept = self.world.getsubmap(self)
        self.expandperceptmap()
        if self.stoppingcriterion():
            return 'Explored'
        if len(robots) > 0:
            for relativepos,robot in robots:
                self.stitchmaps(relativepos,robot)
        self.perceptmap[self.xmapposition,self.ymapposition]=utils.MAPREP.SELF

    def getKey(self,item):
        return item[0]

    def greedymigmove(self):
        #Move greedily MIG
        robots, self.currentPercept = self.world.getsubmap(self)
        self.expandperceptmap()
        if len(robots) > 0:
            for relativepos,robot in robots:
                self.stitchmaps(relativepos,robot)
        if self.stoppingcriterion():
            return 'Explored'
        #options=self.perceptmap[self.xmapposition-self.perceptradius:self.xmapposition+self.perceptradius+1,self.ymapposition-self.perceptradius:self.ymapposition+self.perceptradius+1]
        #print options
        options=[]
        possiblemoves = self.getPossibleMoves()
        for move in possiblemoves:
            i,j = direction[move]
            option = self.perceptmap[self.xmapposition+i-self.perceptradius:self.xmapposition+i+self.perceptradius+1,self.ymapposition+j-self.perceptradius:self.ymapposition+j+self.perceptradius+1]
            options.append([np.count_nonzero(option),(i,j)])
#        for i in range(-self.perceptradius,self.perceptradius+1):
#            for j in range(-self.perceptradius,self.perceptradius+1):
#                if (i!=0 or j!=0) and self.currentPercept[i+1,j+1]==utils.MAPREP.EMPTY and self.testDiag((i,j)):
#                    option = self.perceptmap[self.xmapposition+i-self.perceptradius:self.xmapposition+i+self.perceptradius+1,self.ymapposition+j-self.perceptradius:self.ymapposition+j+self.perceptradius+1]
#                    options.append([np.count_nonzero(option),(i,j)])
        options=sorted(options, key=self.getKey)
        #print options
        if options != []:
            if options[0][0] <(1+2*self.perceptradius)*(1+2*self.perceptradius): 
                direct = (options[0][1][0],options[0][1][1])
            else:
                return self.bayesMove()
            #print direct
            self.move(direct)

    def stoppingcriterion(self):
        #print "entering"
        maptolookup = self.perceptmap#[self.minxposition-1: self.maxxposition+2, self.minyposition-1:self.maxyposition+2 ]
        currentxposition = self.xmapposition#-self.minxposition+1
        currentyposition = self.ymapposition#-self.minyposition+1
        #print currentxposition,currentyposition
        shapeoftheworld=maptolookup.shape
        return self.floodfill(copy.deepcopy(maptolookup), currentxposition, currentyposition, shapeoftheworld)

    def floodfill(self, worldmap, x, y, shapeoftheperceptworld):
        ret=True
        
        if worldmap[x,y]!=utils.MAPREP.BLOCKED:
            
            if worldmap[x,y]==utils.MAPREP.UNEXPLORED:
                return False
            else:
                worldmap[x,y]=utils.MAPREP.BLOCKED
                if x-1>=0:
                    ret=self.floodfill(worldmap, x-1, y, shapeoftheperceptworld)
                if ret and x+1<shapeoftheperceptworld[0]:
                    ret = ret and self.floodfill(worldmap, x+1, y, shapeoftheperceptworld)
                if ret and y-1>=0:
                    ret = ret and self.floodfill(worldmap, x, y-1, shapeoftheperceptworld)
                if ret  and y+1<shapeoftheperceptworld[1]:
                    ret = ret and self.floodfill(worldmap, x, y+1, shapeoftheperceptworld)
                return ret
        return True
