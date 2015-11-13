import os
import robot
import world
import numpy as np
import copy
import utils
import graphics
import Tkinter as tk

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
showmap=graphics.Graphics(len(worldmap))
showmap.creategraphics(world.worldmap, world.robotsbypos[(3,1)].perceptmap, 1)
showmap.root.mainloop()

world.robotsbypos[(3,1)].move(utils.MOVES.NORTH)
world.robotsbypos[(2,1)].move(utils.MOVES.NORTH)
#print world.robotsbypos[1,1].perceptmap
