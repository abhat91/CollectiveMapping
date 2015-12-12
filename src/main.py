import os
import robot
import world
import numpy as np
import copy
import utils
import graphics
import Tkinter as tk
import random
import heapq
import sys

def convertToInteger(multidimentionalList):
    """Converts to integer"""
    return map(int, multidimentionalList)

def readmap():
    """Method to read the map"""
    directoryPath=os.getcwd()
    fname=directoryPath+'/../worlds/map100_100.txt'
    data = [line.rstrip('\n') for line in open(fname)]
    if len(data)==1:
    	data[0]=data[0].replace('\t', ' ')
    	data=[line for line in data[0].split('\r')]
    data = [ map(str,line.split(' ')) for line in data ]
    return map(convertToInteger, data)

def run(t):
    global flag
    global selectedrobot
    global previousRobot
    global world
    if len(showmap.listbox.curselection()) > 0:
        selectedrobot = int(showmap.listbox.curselection()[0])
    if flag==False:
        t = t + 1
        for i in range(len(world.posbyrobots.keys())):
            robot = world.posbyrobots.keys()[i]
            robot.updated = False
        for i in range(len(world.posbyrobots.keys())):
            robot = world.posbyrobots.keys()[i]

            if robot.greedymigmove()=='Explored':
                flag=True
                selectedrobot = i
                print "explored using ",i, "in", t
                return
        if t%100==0:
        	asd=[]
        	for h in range(len(world.posbyrobots.keys())):
        		robot=world.posbyrobots.keys()[i]
        		asd=asd+[np.count_nonzero(robot.perceptmap)]
        	maxima=heapq.nlargest(1, asd)
        	print t, ',', maxima[0]
    if previousRobot != selectedrobot:
    	if t%100==0 or flag==True:
        	showmap.root.after(2,showmap.updateNewRobotMap,world.posbyrobots.keys()[selectedrobot],selectedrobot)
        previousRobot = selectedrobot
    else:
    	if t%100==0 or flag==True:
        	showmap.root.after(2,showmap.updateRobotMap,world.posbyrobots.keys()[selectedrobot],selectedrobot)
    if t%100==0 or flag==True:
    	showmap.root.after(2,showmap.updateGraphics,world.worldmap,world.posbyrobots.keys()[selectedrobot],world.posbyrobots[world.posbyrobots.keys()[selectedrobot]],selectedrobot,t)
    showmap.root.after(200,run,t)
sys.setrecursionlimit(10000)
listofbehaviours=[robot.Robot.gradientmove, robot.Robot.randomMove]
selectedrobot = 0
previousRobot = 0
positions=[]
for i in range(10):
	positions=positions+[(random.random(),random.random())]
worldmap=readmap()
shapeoftworld=np.shape(worldmap)
actualpositions=[]
for x in positions:
	actualpositions=actualpositions+[(int(shapeoftworld[0]*x[0]), int(shapeoftworld[1]*x[1]))]
print actualpositions
world=world.World(np.array(worldmap), [(22, 11)])
for robot in world.posbyrobots.keys():
    robots, robot.currentPercept = world.getsubmap(robot)
    robot.expandperceptmap()
showmap=graphics.Graphics(len(worldmap))
for i in range(0,len(world.robotsbypos)):
    showmap.listbox.insert(tk.END,str(i))

showmap.listbox.activate(0)
flag=False
showmap.creategraphics(world.worldmap, world.posbyrobots.keys()[selectedrobot], selectedrobot)

showmap.root.after(1,run, 0)
showmap.root.mainloop()