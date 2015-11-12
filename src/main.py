import os
import robot
import world
import numpy as np
import copy

def convertToInteger(multidimentionalList):
    return map(int, multidimentionalList)

def readmap():
    directoryPath=os.getcwd()
    fname=directoryPath+'/../worlds/world.txt'
    data = [line.rstrip('\n') for line in open(fname)]
    data = [ map(str,line.split(' ')) for line in data ]
    return map(convertToInteger, data)


worldmap=readmap()
world=world.World(np.array(worldmap), [(4,2), (3,1)])

otherrobotpositionandbot,perceptmap=world.getsubmap(world.robotsbypos[(3,1)])
world.robotsbypos[(3,1)].expandperceptmap(copy.deepcopy(perceptmap))

otherrobotpositionandbot,perceptmap2= world.getsubmap(world.robotsbypos[(4,2)])

world.robotsbypos[(4,2)].expandperceptmap(copy.deepcopy(perceptmap2))
print world.robotsbypos[(3,1)].perceptmap
print '##############################################'
print world.robotsbypos[4,2].perceptmap

