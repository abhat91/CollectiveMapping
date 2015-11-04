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

    def moveRobot(oldpos,newpos):
        robots[newpos] = robots[oldpos].pop()
