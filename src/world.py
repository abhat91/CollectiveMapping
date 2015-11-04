import numpy as np
import tables

class world:
    worldmap = np.zeros(0)
    robots = {}
    ''' Initializer for class world
        @author: renatogg
        map: a numpy matrix to serve as a map, where:
            0 means empty space,
            1 means blocked space,
            2 means there is another robot in that space
        robotpositions: a list of coordinates where to insert the robots
    '''
    def __init__(self,worldmap,robotpositions):
        self.worldmap = worldmap
        for i in robotpositions:
            self.placenewrobot(i)
        return

    ''' Places a new robot in given position
        @author: renatogg
        if position is not available(has another robot or a wall), try to place robot in 8-connected radius from specified position
    '''
    def placenewrobot(self,position):
        if self.worldmap[positon] ==0:
            robots[position] = robot() #Create new robot at given positon
        else:
            radius = 1
            x,y = position
            mx,my = self.worldmap.shape #max values for x and y
            while True:
                for i in range(-radius,radius):#top wall
                    if x+i >= 0 and x+i < mx and y + radius < my:
                        if self.worldmap[x+i,y+radius] == 0:
                            robots[(x+i,y+radius)] = robot()
                            return
                for j in range(-radius,radius):#Right wall
                    if y+j >= 0 and y+j < my and x + radius < mx:
                        if self.worldmap[x+radius,y+j] == 0:
                            robots[(x+radius,y+j)] = robot()
                            return
                for i in range(-radius,radius):#Bottom wall
                    if x+i >= 0 and x+i < mx and y - radius >= 0:
                        if self.worldmap[x+i,y-radius] == 0:
                            robots[(x+i,y-radius)] = robot()
                            return
                for j in range(-radius,radius):#Left  wall
                    if y+j >= 0 and y+j < my and x + radius >= 0:
                        if self.worldmap[x-radius,y+j] == 0:
                            robots[(x-radius,y+j)] = robot()
                            return
                radius +=1

    #Given the current location of the robot and the percept radius, the func tion returns a list of robots in its area
    #and the submap of the percept sequence
    def getsubmap(self, currentlocation, perceptradius):
        """Given the current location of the robot and the percept radius, the func tion returns a
        list of robots in its area and the submap of the percept sequence"""
        northwestlocationx=currentlocation[0]-perceptradius
        northwestlocationy=currentlocation[1]-perceptradius
        currentPercept=self.worldmap[np.ix_([northwestlocationx,northwestlocationx+(2*perceptradius)],[northwestlocationy,northwestlocationy+(2*perceptradius)])]
        #TODO: Change this based on the convention
        currentPercept[perceptradius, perceptradius]=3
        listofrobotlocations=self.getlistofrobotinsubmap(currentPercept)
        if len(listofrobotlocations)>0:
            listofrobots=[]
            for robotlocation in listofrobotlocations:
                listofrobots=listofrobots+[(northwestlocationx+robotlocation[0], northwestlocationy+robotlocation[1]), robots[(northwestlocationx+robotlocation[0], northwestlocationy+robotlocation[1])]]
        return listofrobots, currentPercept

    #Given the submap, it returns the index of the robots in the map
    def getlistofrobotinsubmap(self, submap):
        """Given the map, it returns the index of the robots in the map"""
        robotpositions=[]
        for i in len(submap):
            for j in len(submap):
                #TODO: Robot exists is assumed to be 2
                if submap[i][j]==2:
                    robotpositions=robotpositions+[(i,j)]
        return robotpositions
