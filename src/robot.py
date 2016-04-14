import numpy as np
import utils
import random
import copy
import sys
from heapq import *
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
    def __init__(self,world,maplen,startpos,ID,alg):
        """Initialization method of the robot class. Updates the x and y positions. Also updates the minimum and maximum x and y
        positions"""
        l = maplen
        self.id = ID
        self.startpos = startpos #USED ONLY TO PLOT ALL ROBOTS INTO SCREEN
        self.l=l
        self.perceptmap = np.zeros( shape= (2*l, 2*l), dtype=int)
        self.currentPercept = np.zeros((self.perceptradius*2+1,self.perceptradius*2+1))
        self.xmapposition = l
        self.ymapposition = l


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

        #Variables for goal check
        self.goalList = []

        if alg == 0:
            self.robotStep = self.randomMove;
        elif alg == 1:
            self.robotStep = self.bayesMove;
        elif alg == 2:
            self.robotStep = self.gradientmove;
        elif alg == 3:
            self.robotStep = self.aStar2Move;
        elif alg == 4:
            self.robotStep = self.greedymigmove;



    def getSortedGoals(self):
        orderedGoals = []

        for goal in self.goalList:
            x = goal[0]
            y = goal[1]
            dist = (1.5*self.calcdistance((self.xmapposition,self.ymapposition),(x,y))+self.calcinformation((x,y)))
            heappush(orderedGoals,(dist,goal))
        return orderedGoals

    def aStar2Move(self):
        self.updatePercepts()
        if self.stoppingcriterion():
            return 'Explored'
        if len(self.path) > 0: # If there's a path planned
            nextMove = self.path.pop(0)
            ##print nextMove
            direction = (nextMove[0]-self.xmapposition,nextMove[1]-self.ymapposition)
            if abs(direction[0])>1 or abs(direction[1])>1:
                self.path = []
            else:
                if self.move(direction): # If can follow the plan
                    # robots, self.currentPercept = self.world.getsubmap(self)
                    # ##print robotslist
                    # if len(robots) > 0:
                    #     for relativepos,robot in robots:
                    #         self.stitchmaps(relativepos,robot)
                    return
        # If there's no plan or plan cannot be executed:
        self.path = []
        self.updateExploringTargets()
        for target in self.migfrontiers: # Orderly go through  targets until a path is made
            astar = astar2.Astar2(self.perceptmap,self.currentPercept,(self.xmapposition,self.ymapposition),target)
            path = astar.search()
            if path:
                self.path = path
                ##print path
                break;
            else:
                goal = target[1]
                for i in range(-1,2):
                    for j in range(-1,2):
                        if i!=0 or j!=0:
                            if self.perceptmap[goal[0]+i,goal[1]+j] == utils.MAPREP.EMPTY:
                                target =(target[0], (goal[0]+i,goal[1]+j))
                                break
                    else:
                        continue
                    break
                astar = astar2.Astar2(self.perceptmap,self.currentPercept,(self.xmapposition,self.ymapposition),target)
                path = astar.search()
                if path:
                    self.path = path
                    nextMove = self.path.pop(0)
                    ##print nextMove
                    direction = (nextMove[0]-self.xmapposition,nextMove[1]-self.ymapposition)
                    self.move(direction)
                    ##print path
                    break;
        if len(self.path) == 0: #no possible move for any target, move randomly
            return False

    def getPossibleGoals(self,position):
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

    def addGoals(self,position):
        for i in range(-1,2):
            for j in range(-1,2):
                # #print i,j
                if j!=0 or i!=0:
                    goal = position[0]+i,position[1]+j
                    # #print len(self.perceptmap),position,goal,self.perceptmap[goal]
                    # if self.perceptmap[goal] == utils.MAPREP.UNEXPLORED:
                    #     # if abs(i) == 1 and abs(j) == 1:#if direction is diagonal
                    #     #     if self.perceptmap[position[0]+i,position[1]] == utils.MAPREP.EMPTY  and self.perceptmap[position[0],position[1]+j] == utils.MAPREP.EMPTY:
                    #     #         if goal not in self.goalList:
                    #     #             self.goalList.append(goal)
                    #     # #
                    #     # else:
                    if goal not in self.goalList:
                        self.goalList.append(goal)

    def expandperceptmap(self):
        """Given the percept matrix, the robot adds the percept to the map of the robot"""
        mapstartX = self.xmapposition - (self.perceptradius)
        mapstartY = self.ymapposition- ( self.perceptradius)
        #Adds the percepts to the robot's map of the world
        # #print self.currentPercept

        for i in range( (2*self.perceptradius) + 1 ):
            for j in range( (2*self.perceptradius) + 1 ):
                #Add Goals to the goallist
                pos = (mapstartX+i,mapstartY+j)
                if pos != (self.xmapposition,self.ymapposition):#and self.perceptmap[pos] == utils.MAPREP.UNEXPLORED:
                    if self.currentPercept[i,j] != utils.MAPREP.BLOCKED:
                        if i == 1 or j==1:#Not diagonal
                            self.addGoals(pos)
                        else:#Diagonal
                            # #print pos,self.currentPercept[i,1],self.currentPercept[1,j]
                            if self.currentPercept[i,1]!=utils.MAPREP.BLOCKED or self.currentPercept[1,j]  != utils.MAPREP.BLOCKED:
                                # #print "added"
                                self.addGoals(pos)


        for i in range( (2*self.perceptradius) + 1 ):
            for j in range( (2*self.perceptradius) + 1 ):
                pos = (mapstartX+i,mapstartY+j)
                self.perceptmap[pos] = copy.deepcopy(self.currentPercept[i,j])

        self.perceptmap[self.xmapposition,self.ymapposition] = utils.MAPREP.SELF
        self.perceptmap[self.perceptmap == utils.MAPREP.PEER] = utils.MAPREP.EMPTY
        # #print self.goalList

    #Given 2 robots in proximity, the map of the other robot is taken and stitched to the current map to make a larger map of the environment
    def stitchmaps(self, relativepos, robot):
        """Given 2 robots in proximity, the map of the other robot is taken and stitched to the current map to make a larger map of the environment"""
        minrobot = [min(np.nonzero(robot.perceptmap)[0]),min(np.nonzero(robot.perceptmap)[1])]
        maxrobot = [max(np.nonzero(robot.perceptmap)[0])+1,max(np.nonzero(robot.perceptmap)[1])+1]
        minself = [min(np.nonzero(self.perceptmap)[0]),min(np.nonzero(self.perceptmap)[1])]
        rpositionOfrobotX = robot.xmapposition - minrobot[0]
        rpositionOfrobotY = robot.ymapposition - minrobot[1] #robot.minyposition
        robotmap = copy.deepcopy(robot.perceptmap[ minrobot[0]:maxrobot[0], minrobot[1]:maxrobot[1]])
        robotmap[robotmap == utils.MAPREP.SELF] = utils.MAPREP.EMPTY
        robotmap[robotmap == utils.MAPREP.PEER] = utils.MAPREP.EMPTY
        shapeofworld = np.shape(robotmap)
        positionxofrobot = self.xmapposition-self.perceptradius+relativepos[0]
        positionyofrobot = self.ymapposition-self.perceptradius+relativepos[1]
        startmapx = positionxofrobot-rpositionOfrobotX
        startmapy = positionyofrobot-rpositionOfrobotY
        # #print self.id,robot.id
        # mapearea = ()
        # #print startmapx,startmapx+robotmap.shape[0]
        # #print startmapy,startmapy+robotmap.shape[1]
        submap = copy.deepcopy(self.perceptmap[startmapx:startmapx+robotmap.shape[0],
                                               startmapy:startmapy+robotmap.shape[1]])

        # #print submap.shape,robotmap.shape

        newsubmap = np.maximum(submap,robotmap)
        # newsubmap = np.maximum.reduce([self.perceptmap[max(0,startmapx):min(startmapx+shapeofworld[0],len(self.perceptmap)),
        # max(0,startmapy):min(startmapy+shapeofworld[1],len(self.perceptmap))], robotmap])
        self.perceptmap[startmapx:startmapx+robotmap.shape[0],
                        startmapy:startmapy+robotmap.shape[1]] = copy.deepcopy(newsubmap)
        self.perceptmap[self.xmapposition, self.ymapposition]=utils.MAPREP.SELF
        rPosX = robot.xmapposition


        # raw_input("end stitch")
    def move(self,dir):
        """Moves the robot by one position and updates the map"""
        if self.world.robotMove(self, dir):
            self.perceptmap[self.xmapposition,self.ymapposition]=utils.MAPREP.EMPTY
        #move successful, Update percept map
            self.xmapposition += dir[0]
            self.ymapposition += dir[1]
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
            ##print possiblemoves
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
        robots, self.currentPercept = self.world.getsubmap(self)
        self.expandperceptmap()

        if len(robots) > 0:
            for relativepos,robot in robots:
                self.stitchmaps(relativepos,robot)
            for relativepos,robot in robots:

                revrelativepos = (abs(relativepos[0]-2),abs(relativepos[1]-2))
                # #print relativepos
                # raw_input("reverse stitch")
                robot.reverseStitch(revrelativepos,self)
            for relativepos,robot in robots:
                goals = copy.deepcopy(robot.goalList)
                #robots2, robot.currentPercept = robot.world.getsubmap(robot)
                #robot.expandperceptmap()
                for goal1 in goals:
                    # #print (self.l - self.xmapposition), relativepos[0]-1,(self.l - robot.xmapposition)
                    gg =copy.deepcopy(((goal1[0] - ((self.l - self.xmapposition) - (relativepos[0]-1)-(robot.l - robot.xmapposition))),
                        (goal1[1] - ((self.l - self.ymapposition) - (relativepos[1]-1)-(robot.l - robot.ymapposition)))))
                    # if g not in self.goalList:
                    #     # #print g,goal
                    self.goalList.append(gg);

        self.goalList = list(set([g for g in self.goalList if self.perceptmap[g]==utils.MAPREP.UNEXPLORED]))
    def reverseStitch(self,relativepos,robot):
        robots, self.currentPercept = self.world.getsubmap(self)
        self.expandperceptmap()
        if len(robots) > 0:
            for relativepos,robot in robots:
                self.stitchmaps(relativepos,robot)

                #robots2, robot.currentPercept = robot.world.getsubmap(robot)
                #robot.expandperceptmap()
            for relativepos,robot in robots:
                goals = copy.deepcopy(robot.goalList)
                for goal1 in goals:
                    # #print (self.l - self.xmapposition), relativepos[0]-1,(self.l - robot.xmapposition)
                    gg =copy.deepcopy(((goal1[0] - ((self.l - self.xmapposition) - (relativepos[0]-1)-(robot.l - robot.xmapposition))),
                        (goal1[1] - ((self.l - self.ymapposition) - (relativepos[1]-1)-(robot.l - robot.ymapposition)))))
                    # if g not in self.goalList:
                    #     # #print g,goal
                    self.goalList.append(gg);
        self.goalList = list(set([g for g in self.goalList if self.perceptmap[g]==utils.MAPREP.UNEXPLORED]))

                    # robot.updatePercepts()


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
        minself = [min(np.nonzero(self.perceptmap)[0]),min(np.nonzero(self.perceptmap)[1])]
        maxself = [max(np.nonzero(self.perceptmap)[0]),max(np.nonzero(self.perceptmap)[1])]
        sizeofsubblockx=(maxself[0]-minself[0])/2
        sizeofsubblocky=(maxself[1]-minself[1])/2
        northwest=self.perceptmap[minself[0]:minself[0]+sizeofsubblockx, minself[1]:minself[1]+sizeofsubblocky]
        northeast=self.perceptmap[minself[0]:minself[0]+sizeofsubblockx, minself[1]+sizeofsubblocky:maxself[1]]
        southwest=self.perceptmap[minself[0]+sizeofsubblockx:maxself[0], minself[1]:minself[1]+sizeofsubblocky]
        southeast=self.perceptmap[minself[0]+sizeofsubblockx:maxself[0], minself[1]+sizeofsubblocky: maxself[1]]
        voronoi=[(np.size(northwest)-np.count_nonzero(northwest))/float(np.size(northwest)+np.size(northeast)+np.size(southwest)+np.size(southeast)), (np.size(northeast)-np.count_nonzero(northeast))/float(np.size(northwest)+np.size(northeast)+np.size(southwest)+np.size(southeast)), (np.size(northeast)-np.count_nonzero(southwest))/float(np.size(northwest)+np.size(northeast)+np.size(southwest)+np.size(southeast)), (np.size(northeast)-np.count_nonzero(southeast))/float(np.size(northwest)+np.size(northeast)+np.size(southwest)+np.size(southeast))]
        maxima= nlargest(number, voronoi)
        return voronoi.index(maxima[number-1])

    def gradientmove(self):
        self.updatePercepts()
        if self.stoppingcriterion():
            return 'Explored'
        robots, self.currentPercept = self.world.getsubmap(self)
        if len(robots) > 0:
            for relativepos,robot in robots:
                robot.previousMove=random.choice(self.getoppositedirection(self.previousMove))
                # self.stitchmaps(relativepos,robot)
        if self.previousMove==None or self.previousMove==0:
    		self.previousMove=utils.MOVES.NORTH
        probabilityofmove=0.4
        randommove=0.051
        self.sidetomove=self.getleastmappedarea(self.prioritysearch)
        x=0


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

        self.perceptmap[self.xmapposition,self.ymapposition]=utils.MAPREP.SELF
        # self.findclosestunexploredpoint()

    def getKey(self,item):
        return item[0]

    def greedymigmove(self):
        #Move greedily MIG
        self.updatePercepts()
        if self.stoppingcriterion():
            return 'Explored'
        #options=self.perceptmap[self.xmapposition-self.perceptradius:self.xmapposition+self.perceptradius+1,self.ymapposition-self.perceptradius:self.ymapposition+self.perceptradius+1]
        ##print options
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
        ##print options
        if options != []:
            if options[0][0] <(1+2*self.perceptradius)*(1+2*self.perceptradius):
                self.previousMove=options[0][1]
                direct = (options[0][2][0],options[0][2][1])
                self.move(direct)
            else:
                return self.aStar2Move()

        else:
            return self.aStar2Move()
            ##print direct



    def updateExploringTargets(self):
        self.migfrontiers = []
        # self.floodfillfrontiers(copy.deepcopy(self.perceptmap),self.xmapposition,self.ymapposition)
        self.migfrontiers = self.getSortedGoals()




    def calcdistance(self,point,goal):
        dx = abs(point[0]-goal[0])
        dy = abs(point[1]-goal[1])
        #dist = 1*(dx + dy)
        dist = (math.sqrt(dx**2+dy**2))
        return dist

    def calcinformation(self,loc):
        return np.count_nonzero(self.perceptmap[loc[0]-self.perceptradius:loc[0]+self.perceptradius+1,loc[1]-self.perceptradius:loc[1]+self.perceptradius+1])


    def stoppingcriterion(self):
        # ##print "entering"
        # maptolookup = self.perceptmap#[self.minxposition-1: self.maxself[0]+2, self.minyposition-1:self.maxself[1]+2 ]
        # currentxposition = self.xmapposition#-self.minxposition+1
        # currentyposition = self.ymapposition#-self.minyposition+1
        # ##print currentxposition,currentyposition
        # shapeoftheworld=maptolookup.shape
        # return self.floodfill(copy.deepcopy(maptolookup), currentxposition, currentyposition, shapeoftheworld)
        return len(self.goalList) == 0
