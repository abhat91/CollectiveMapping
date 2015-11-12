import os
import robot
import world
import numpy as np
import copy
import utils

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
world.robotsbypos[(4,2)].stitchmaps(otherrobotpositionandbot[0][0],otherrobotpositionandbot[0][1])

world.robotMove(world.robotsbypos[(4,2)], utils.MOVES.NORTH)
otherrobotpositionandbot,perceptmap=world.getsubmap(world.robotsbypos[(3,2)])
world.robotsbypos[(3,2)].expandperceptmap(copy.deepcopy(perceptmap))
print world.robotsbypos[3,2].perceptmap
