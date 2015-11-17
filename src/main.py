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
    """Converts to integer"""
    return map(int, multidimentionalList)

def readmap():
    """Method to read the map"""
    directoryPath=os.getcwd()
    fname=directoryPath+'/../worlds/world3.txt'
    data = [line.rstrip('\n') for line in open(fname)]
    data = [ map(str,line.split(' ')) for line in data ]
    return map(convertToInteger, data)


selectedrobot = 0

worldmap=readmap()
world=world.World(np.array(worldmap), [(5,3),(5,4),(15,15),(15,1),(1,15),(1,16),(1,17),(2,15),(2,15),(0,15),(0,5),(11,15),(12,15),(12,13),(1,15),(14,1),(17,1),(15,15),(1,1)])
showmap=graphics.Graphics(len(worldmap))
for i in range(0,len(world.robotsbypos)):
    showmap.listbox.insert(tk.END,str(i))

showmap.listbox.activate(0)
#print world.robotsbypos[1,1].perceptmap
flag=False
def run(t,worldmap,robotmap,robotid):
    global flag
    if flag==False:
        t = t + 1
        for robot in world.posbyrobots.keys():
            if robot.randomMove()=='Explored':
                flag=True

    selectedrobot = 0
    if len(showmap.listbox.curselection()) > 0:
        selectedrobot = int(showmap.listbox.curselection()[0])
    showmap.root.after(10,run, t,world.worldmap,world.posbyrobots.keys()[selectedrobot].perceptmap,selectedrobot)
    showmap.root.after(1,showmap.updateGraphics,worldmap,robotmap,robotid,t)


showmap.creategraphics(world.worldmap, world.posbyrobots.keys()[selectedrobot].perceptmap, selectedrobot)
showmap.root.after(1,run, 0,world.worldmap,world.posbyrobots.keys()[selectedrobot].perceptmap,selectedrobot)
showmap.root.mainloop()
