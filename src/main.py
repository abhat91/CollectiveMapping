import os
import robot
import world
import numpy as np
import copy
import utils
import graphics
import Tkinter as tk
import random

def convertToInteger(multidimentionalList):
    return map(int, multidimentionalList)

def readmap():
    directoryPath=os.getcwd()
    fname=directoryPath+'/../worlds/world3.txt'
    data = [line.rstrip('\n') for line in open(fname)]
    data = [ map(str,line.split(' ')) for line in data ]
    return map(convertToInteger, data)


selectedrobot = 0

worldmap=readmap()
world=world.World(np.array(worldmap), [(5,3),(5,4),(15,15),(15,1),(1,15)])
showmap=graphics.Graphics(len(worldmap))
for i in range(0,len(world.robotsbypos)):
    showmap.listbox.insert(tk.END,str(i))

showmap.listbox.activate(0)
#print world.robotsbypos[1,1].perceptmap
def run(t,worldmap,robotmap,robotid):
    t = t + 1
    #timetext.set('Time: ' + str(t))
    #worldmap = np.random.randint(4, size=(size,size)) -1
    for robot in world.posbyrobots.keys():
        robot.randomMove()

    selectedrobot = 0
    if len(showmap.listbox.curselection()) > 0:
        selectedrobot = showmap.listbox.curselection()[0]
    showmap.root.after(10,run, t,world.worldmap,world.posbyrobots.keys()[selectedrobot].perceptmap,selectedrobot)
    showmap.root.after(1,showmap.updateGraphics,worldmap,robotmap,robotid,t)


showmap.creategraphics(world.worldmap, world.posbyrobots.keys()[selectedrobot].perceptmap, selectedrobot)
showmap.root.after(1,run, 0,world.worldmap,world.posbyrobots.keys()[selectedrobot].perceptmap,selectedrobot)
showmap.root.mainloop()
