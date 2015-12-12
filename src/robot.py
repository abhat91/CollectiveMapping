import numpy as np
import utils
import random
import copy
import sys
import heapq
import astar
import math
import astar2
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
xval=0
yval=0
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
        self.previousMove = 0
        self.sidetomove=0
        self.updated = False
        self.unchangedvaluecount=0
        self.prioritysearch=1

        #Variables for A* algorithm
        self.xdestination=0
        self.ydestination=0
        self.isastar=False
        self.astarduration=0

        # Variables for A* 2 algorithm
        self.destination = (0,0)
        self.path = []

    def aStar2Move(self):
        self.updatePercepts()
        if self.stoppingcriterion():
            return 'Explored'
        if len(self.path) > 0: # If there's a path planned
            nextMove = self.path.pop(0)
            #print nextMove
            direction = (nextMove[0]-self.xmapposition,nextMove[1]-self.ymapposition)
            if abs(direction[0])>1 or abs(direction[1])>1:
                self.path = []
            else:
                if self.move(direction): # If can follow the plan
                    robots, self.currentPercept = self.world.getsubmap(self)
                    #print robotslist
                    if len(robots) > 0:
                        for relativepos,robot in robots:
                            self.stitchmaps(relativepos,robot)
                    return
        # If there's no plan or plan cannot be executed:
        self.path = []
        self.updateExploringTargets()
        for target in self.migfrontiers: # Orderly go through  targets until a path is made
            astar = astar2.Astar2(self.perceptmap,self.currentPercept,(self.xmapposition,self.ymapposition),target)
            path = astar.search()
            if path:
                self.path = path
                #print path
                break;
        if len(self.path) == 0: #no possible move for any target, move randomly
            self.randomMove()


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
            return True
        return False
    def bayesMove(self):
        '''@author:renatogg - Moves to a direction with probability given by previous movement'''
        self.updatePercepts()
        if self.stoppingcriterion():
            return 'Explored'
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
        self.updatePercepts()
        if self.stoppingcriterion():
            return 'Explored'
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

    def updatePercepts(self):
        if (not self.updated):
            robots, self.currentPercept = self.world.getsubmap(self)
            self.expandperceptmap()
            self.updated = True
            if len(robots) > 0:
                for relativepos,robot in robots:
                    self.stitchmaps(relativepos,robot)
                    robot.updatePercepts()


    def getoppositedirection(self, currentdirection):
        directions={utils.MOVES.NORTH:[utils.MOVES.SOUTHEAST,utils.MOVES.SOUTH,utils.MOVES.SOUTHWEST],
                    utils.MOVES.SOUTH:[utils.MOVES.NORTH, utils.MOVES.NORTHWEST,utils.MOVES.NORTHEAST],
                    utils.MOVES.EAST:[utils.MOVES.WEST,utils.MOVES.NORTHWEST, utils.MOVES.SOUTHWEST],
                    utils.MOVES.WEST:[utils.MOVES.EAST,utils.MOVES.NORTHEAST, utils.MOVES.SOUTHEAST],
                    utils.MOVES.NORTHEAST:[utils.MOVES.SOUTH,utils.MOVES.SOUTHWEST,utils.MOVES.SOUTHEAST],
                    utils.MOVES.NORTHWEST:[utils.MOVES.SOUTH, utils.MOVES.SOUTHEAST,utils.MOVES.SOUTHWEST],
                    utils.MOVES.SOUTHEAST:[utils.MOVES.NORTHWEST,utils.MOVES.WEST,utils.MOVES.NORTH],
                    utils.MOVES.SOUTHWEST:[utils.MOVES.NORTHEAST,utils.MOVES.EAST,utils.MOVES.NORTH]}
        #dirforotherrobot=self.getleastmappedarea(2)
        return directions[currentdirection]

    def getleastmappedarea(self, number):
        sizeofsubblockx=(self.maxxposition-self.minxposition)/2
        sizeofsubblocky=(self.maxyposition-self.minyposition)/2
        northwest=self.perceptmap[self.minxposition:self.minxposition+sizeofsubblockx, self.minyposition:self.minyposition+sizeofsubblocky]
        northeast=self.perceptmap[self.minxposition:self.minxposition+sizeofsubblockx, self.minyposition+sizeofsubblocky:self.maxyposition]
        southwest=self.perceptmap[self.minxposition+sizeofsubblockx:self.maxxposition, self.minyposition:self.minyposition+sizeofsubblocky]
        southeast=self.perceptmap[self.minxposition+sizeofsubblockx:self.maxxposition, self.minyposition+sizeofsubblocky: self.maxyposition]
        voronoi=[(np.size(northwest)-np.count_nonzero(northwest))/float(np.size(northwest)+np.size(northeast)+np.size(southwest)+np.size(southeast)), (np.size(northeast)-np.count_nonzero(northeast))/float(np.size(northwest)+np.size(northeast)+np.size(southwest)+np.size(southeast)), (np.size(northeast)-np.count_nonzero(southwest))/float(np.size(northwest)+np.size(northeast)+np.size(southwest)+np.size(southeast)), (np.size(northeast)-np.count_nonzero(southeast))/float(np.size(northwest)+np.size(northeast)+np.size(southwest)+np.size(southeast))]
        maxima=heapq.nlargest(number, voronoi)
        return voronoi.index(maxima[number-1])

    def gradientmove(self):
    	if self.previousMove==None or self.previousMove==0:
    		self.previousMove=utils.MOVES.NORTH
        probabilityofmove=0.4
        randommove=0.051
        self.sidetomove=self.getleastmappedarea(self.prioritysearch)
        x=0
        robots, self.currentPercept = self.world.getsubmap(self)
        if len(robots) > 0:
            for relativepos,robot in robots:
                robot.previousMove=random.choice(self.getoppositedirection(self.previousMove))
                self.stitchmaps(relativepos,robot)
        while True:
            if self.sidetomove==0:
                if random.random()<probabilityofmove:
                    x=[utils.MOVES.NORTHWEST, utils.MOVES.NORTH, utils.MOVES.WEST]
                else:
                    x=[utils.MOVES.NORTHEAST, utils.MOVES.EAST, utils.MOVES.SOUTHEAST, utils.MOVES.SOUTH, utils.MOVES.SOUTHWEST]
            elif self.sidetomove==1:
                if random.random()<probabilityofmove:
                    x=[utils.MOVES.NORTHEAST, utils.MOVES.NORTH, utils.MOVES.EAST]
                else:
                    x=[utils.MOVES.NORTHWEST, utils.MOVES.WEST, utils.MOVES.SOUTHEAST, utils.MOVES.SOUTH, utils.MOVES.SOUTHWEST]
            elif self.sidetomove==2:
                if random.random()<probabilityofmove:
                    x=[utils.MOVES.SOUTHWEST, utils.MOVES.SOUTH, utils.MOVES.WEST]
                else:
                    x=[utils.MOVES.NORTH, utils.MOVES.NORTHWEST, utils.MOVES.NORTHEAST, utils.MOVES.EAST, utils.MOVES.SOUTHEAST]
            elif self.sidetomove==3:
                if random.random()<probabilityofmove:
                    x=[utils.MOVES.SOUTHEAST, utils.MOVES.SOUTH, utils.MOVES.EAST]
                else:
                    x=[utils.MOVES.SOUTHWEST, utils.MOVES.WEST, utils.MOVES.NORTHWEST, utils.MOVES.NORTH, utils.MOVES.NORTHEAST]
            movechoice=random.choice(x)
            if self.perceptmap[self.xmapposition+movechoice[0], self.ymapposition+movechoice[1]]!=utils.MAPREP.BLOCKED:
                if self.previousMove==movechoice:
                        self.move(movechoice)
                        self.unchangedvaluecount+=1
                        self.previousMove=movechoice
                        break
                else:
                	self.aStar2Move()
                	self.previousMove=utils.MOVES.NORTH
                	break
                    #self.unchangedvaluecount=0
                    #self.prioritysearch=1
                    #if random.random()<randommove:
                    #    self.move(movechoice)
                    #    self.previousMove=movechoice
                        
        if self.unchangedvaluecount>20:
            self.prioritysearch+=1
        if self.prioritysearch>4:
            self.prioritysearch=1
        robots, self.currentPercept = self.world.getsubmap(self)
        self.expandperceptmap()
        if self.stoppingcriterion():
            return 'Explored'
        if len(robots) > 0:
            for relativepos,robot in robots:
                self.stitchmaps(relativepos,robot)
        self.perceptmap[self.xmapposition,self.ymapposition]=utils.MAPREP.SELF
        self.findclosestunexploredpoint()

    def getKey(self,item):
        return item[0]

    def greedymigmove(self):
        #Move greedily MIG
        self.updatePercepts()
        if self.stoppingcriterion():
            return 'Explored'
        #options=self.perceptmap[self.xmapposition-self.perceptradius:self.xmapposition+self.perceptradius+1,self.ymapposition-self.perceptradius:self.ymapposition+self.perceptradius+1]
        #print options
        options=[]
        possiblemoves = self.getPossibleMoves()
        for move in possiblemoves:
            i,j = direction[move]
            option = self.perceptmap[self.xmapposition+i-self.perceptradius:self.xmapposition+i+self.perceptradius+1,self.ymapposition+j-self.perceptradius:self.ymapposition+j+self.perceptradius+1]
            options.append([np.count_nonzero(option),move,(i,j)])
#        for i in range(-self.perceptradius,self.perceptradius+1):
#            for j in range(-self.perceptradius,self.perceptradius+1):
#                if (i!=0 or j!=0) and self.currentPercept[i+1,j+1]==utils.MAPREP.EMPTY and self.testDiag((i,j)):
#                    option = self.perceptmap[self.xmapposition+i-self.perceptradius:self.xmapposition+i+self.perceptradius+1,self.ymapposition+j-self.perceptradius:self.ymapposition+j+self.perceptradius+1]
#                    options.append([np.count_nonzero(option),(i,j)])
        options=sorted(options, key=self.getKey)
        #print options
        if options != []:
            if options[0][0] <(1+2*self.perceptradius)*(1+2*self.perceptradius):
                self.previousMove=options[0][1]
                direct = (options[0][2][0],options[0][2][1])
                self.move(direct)
            else:
                return self.aStar2Move()

        else:
            return self.aStar2Move()
            #print direct
        

    def updateExploringTargets(self):
        self.migfrontiers = []
        self.floodfillfrontiers(copy.deepcopy(self.perceptmap),self.xmapposition,self.ymapposition)
        self.migfrontiers = sorted(self.migfrontiers, key=self.getKey)



    def targetedmigmove(self):
        robots, self.currentPercept = self.world.getsubmap(self)
        self.expandperceptmap()
        self.migfrontiers = []
        shapeoftheworld=(self.maxxposition, self.maxyposition)
        self.perceptmap[self.perceptmap == 7] = utils.MAPREP.UNEXPLORED
        self.perceptmap[self.perceptmap == 8] = utils.MAPREP.UNEXPLORED
        self.floodfillfrontiers(copy.deepcopy(self.perceptmap),self.xmapposition,self.ymapposition)
        #print self.migfrontiers
        options=sorted(self.migfrontiers, key=self.getKey)

        #for option in options:
        #    self.perceptmap[option[1][0],option[1][1]] = 7
        if len(options) == 0:
            return 'Explored'
        goal = options[0][1]
        #self.perceptmap[goal[0],goal[1]]=8
        #print (self.xmapposition,self.ymapposition),goal

        pt = astar.PathFinder(copy.deepcopy(self.perceptmap),len(self.perceptmap),(self.xmapposition,self.ymapposition),goal)
        path = pt.run()
        direction = (path[0][0]-self.xmapposition,path[0][1]-self.ymapposition)
        #print direction

        '''for option in self.migfrontiers:
            print self.perceptmap[option[1][0],option[1][1]],self.calcdistance((self.xmapposition,self.ymapposition),option)'''

        self.move(direction)
        robots, self.currentPercept = self.world.getsubmap(self)
        #print robotslist
        if len(robots) > 0:
            for relativepos,robot in robots:
                self.stitchmaps(relativepos,robot)

    def calcdistance(self,point,goal):
        dx = abs(point[0]-goal[0])
        dy = abs(point[1]-goal[1])
        #dist = 1*(dx + dy)
        dist = (math.sqrt(pow(dx,2)+pow(dy,2)))
        return dist

    def calcinformation(self,loc):
        return np.count_nonzero(self.perceptmap[loc[0]-self.perceptradius:loc[0]+self.perceptradius+1,loc[1]-self.perceptradius:loc[1]+self.perceptradius+1])

    def floodfillfrontiers(self, worldmap, x, y):
        if worldmap[x,y]!=utils.MAPREP.BLOCKED:
            if worldmap[x,y]==utils.MAPREP.UNEXPLORED:
                worldmap[x,y]=utils.MAPREP.BLOCKED
                #self.migfrontiers.append((self.calcinformation((x,y)),(x,y)))
                self.migfrontiers.append((1.5*self.calcdistance((self.xmapposition,self.ymapposition),(x,y))+self.calcinformation((x,y)),(x,y)))
                return
            if x-1>0:
                worldmap[x,y]=utils.MAPREP.BLOCKED
                self.floodfillfrontiers(worldmap, x-1, y)
            if x+1<len(worldmap):
                worldmap[x,y]=utils.MAPREP.BLOCKED
                self.floodfillfrontiers(worldmap, x+1, y)
            if y+1<len(worldmap):
                worldmap[x,y]=utils.MAPREP.BLOCKED
                self.floodfillfrontiers(worldmap, x, y+1)
            if y-1>0:
                worldmap[x,y]=utils.MAPREP.BLOCKED
                self.floodfillfrontiers(worldmap, x, y-1)
            return
        return

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

    def findclosestunexploredpoint(self):
        maptolookup = self.perceptmap
        currentxposition = self.xmapposition
        currentyposition = self.ymapposition
        shapeoftheworld=maptolookup.shape
        returnedvalue=self.getunfilledpoint(copy.deepcopy(maptolookup), currentxposition, currentyposition, shapeoftheworld)
#        print "Forntier:",self.xdestination, self.ydestination

    def getunfilledpoint(self, worldmap, x, y, shapeoftheperceptworld):
        ret=True
        if worldmap[x,y]!=utils.MAPREP.BLOCKED:
            if worldmap[x,y]==utils.MAPREP.UNEXPLORED:
                self.xdestination=x
                self.ydestination=y
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
                if ret==False:
                    self.xdestination=x
                    self.ydestination=y
                return ret
        return True
