import numpy as np
import robot
import utils
import copy

class World:
    worldmap = np.zeros(0)
    robotsbypos = {}
    posbyrobots = {}

    def __init__(self,worldmap,robotpositions):
        """ Initializer for class world
            @author: renatogg
            map: a numpy matrix to serve as a map, where:
                0 means unexplored,
                1 means the robot that is querying the world is in the space
                2 current robot position
                3 empty,
                4 blocked

            robotpositions: a list of coordinates where to insert the robots
        """
        self.worldmap = worldmap
        for i in robotpositions:
            self.placenewrobot(i)
        return

    def createRobot(self,position):
        """
        Creates a new robot in given position
        """
        self.robotsbypos[position] = robot.Robot(self, len(self.worldmap)) #Create new robot at given positon
        self.posbyrobots[self.robotsbypos[position]] = position
        self.worldmap[position]=utils.MAPREP.PEER
    def placenewrobot(self,position):
        """ Places a new robot in given position
            @author: renatogg
            if position is not available(has another robot or a wall), try to place robot in 8-connected radius from specified position
        """
        if self.worldmap[position] ==utils.MAPREP.EMPTY:
            self.createRobot(position)
        else:
            radius = 1
            x,y = position
            mx,my = self.worldmap.shape #max values for x and y
            while True:
                for i in range(-radius,radius):#top wall
                    if x+i >= 0 and x+i < mx and y + radius < my:
                        if self.worldmap[x+i,y+radius] == utils.MAPREP.EMPTY:
                            self.createRobot((x+i,y+radius))
                            return
                for j in range(-radius,radius):#Right wall
                    if y+j >= 0 and y+j < my and x + radius < mx:
                        if self.worldmap[x+radius,y+j] == utils.MAPREP.EMPTY:
                            self.createRobot((x+radius,y+j))
                            return
                for i in range(-radius,radius):#Bottom wall
                    if x+i >= 0 and x+i < mx and y - radius >= 0:
                        if self.worldmap[x+i,y-radius] == utils.MAPREP.EMPTY:
                            self.createRobot((x+i,y-radius))
                            return
                for j in range(-radius,radius):#Left  wall
                    if y+j >= 0 and y+j < my and x - radius >= 0:
                        if self.worldmap[x-radius,y+j] == utils.MAPREP.EMPTY:
                            self.createRobot((x-radius,y+j))
                            return
                radius +=1

    #Given the current location of the robot and the percept radius, the func tion returns a list of robots in its area
    #and the submap of the percept sequence
    def getsubmap(self, robot):
        """Given the current location of the robot and the percept radius, the
        function returns a list of robots in its area and the submap of the
        percept sequence"""
        currentlocation = self.posbyrobots[robot]
        perceptradius = robot.perceptradius
        currentPercept = None
        #get the top-left point of the square surrounding current robot
        startx=currentlocation[0]-perceptradius
        starty=currentlocation[1]-perceptradius
        #submatrix with size perceptradius*2+1xperceptradius*2+1
        currentPercept=copy.deepcopy(self.worldmap[startx:startx+(2*perceptradius)+1,starty:starty+(2*perceptradius)+1])
        #Add 3 to the position where the robot is (relative to the submap)
        currentPercept[perceptradius, perceptradius]=utils.MAPREP.SELF
        currentPercept[currentPercept == utils.MAPREP.UNEXPLORED] = utils.MAPREP.EMPTY
        robotsrelativelocations=self.getrobotsrelativepositions(currentPercept)
        if len(self.robotsbypos)>0:
            listofrobots=[]
            for robotlocation in robotsrelativelocations:
                listofrobots.append(((robotlocation[0], robotlocation[1]), self.robotsbypos[(startx+robotlocation[0], starty+robotlocation[1])]))
    
    #print "p",currentPercept
        return listofrobots, currentPercept

    #Given the submap, it returns the index of the robots in the map
    def getrobotsrelativepositions(self, submap):
        """Given the map, it returns the relative position of the robots in the submap"""
        robotpositions=[]
        for i in range(0,len(submap)):
            for j in range(0,len(submap)):
                #if a robot is at position i,j:
                if submap[i][j]==utils.MAPREP.PEER:
                    robotpositions.append((i,j))
        return robotpositions

    #Find out if a diagonal move is valid
    def testDiag(self,robot,movement):
        rx,ry = self.posbyrobots[robot]
        dx,dy = movement
        if abs(dx) == 1 and abs(dy) == 1:
            if self.worldmap[rx+dx,ry] == utils.MAPREP.EMPTY and self.worldmap[rx,ry+dy] == utils.MAPREP.EMPTY:
                return True
            else:
                return False
        return True

    def robotMove(self,robot,movement):
        """
        @author renatogg
        Given a robot and a movement request, if the new location is empty (0),
        allow robot to move to new position
        """
        rx,ry = self.posbyrobots[robot]
        dx,dy = movement
        newpos = rx+dx,ry+dy
        if self.testDiag(robot,movement)!=True:
            #print "robot",robot,"attempted to move to diagonal",newpos
            return False
        if self.worldmap[newpos] == utils.MAPREP.EMPTY:
            self.posbyrobots[robot] = newpos
            self.worldmap[newpos] = utils.MAPREP.PEER
            self.worldmap[rx,ry] = utils.MAPREP.EMPTY
            self.robotsbypos[newpos] = self.robotsbypos.pop((rx,ry))
            return True
        #print "robot",robot,"attempted to move to non-empty space",newpos,self.worldmap[newpos]
        return False
