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
import gc

def convertToInteger(multidimentionalList):
    """Converts to integer"""
    return map(int, multidimentionalList)

def readmap():
    """Method to read the map"""
    directoryPath=os.getcwd()
    fname=directoryPath+'/../worlds/world3.txt'
    data = [line.rstrip('\n') for line in open(fname)]
    if len(data)==1:
    	data[0]=data[0].replace('\t', ' ')
    	data=[line for line in data[0].split('\r')]
    print data[0]
    raw_input("Xxx")
    data = [ map(str,line.split(' ')) for line in data ]
    return map(convertToInteger, data)

def run(t):
    global flag
    global selectedrobot
    global previousRobot
    global world
    global robots
    if len(showmap.listbox.curselection()) > 0:# and not flag:
        selectedrobot = int(showmap.listbox.curselection()[0])

    if flag==False:
        t = t + 1
        # for robot in robots:
            # robot.updated = False
        for robot in robots:
            if robot.greedymigmove()=='Explored':

                flag=True
                selectedrobot = robots.index(robot)
                robot.updated = False
                # robot.updatePercepts()
                # showmap.root.after(1,showmap.updateRobotMap,world.posbyrobots.keys()[selectedrobot],selectedrobot,world.posbyrobots.values())

                print "explored using ",selectedrobot, "in", t,"size:",np.count_nonzero(robot.perceptmap)
                # return
            # print t, robot.goalList
            # robot.updated = False
            # robot.updatePercepts()
            # print robot.goalList

        if t%100==0:
        	asd=[]
        	for robot in robots:
        		asd=asd+[np.count_nonzero(robot.perceptmap)]
        	maxima=heapq.nlargest(1, asd)
        	print t, ',', maxima[0]
    if (t > 0):
        # if previousRobot != selectedrobot or flag==True:
        #     showmap.root.after(1,showmap.updateRobotMap,robots[selectedrobot],selectedrobot,world.posbyrobots.values())
        #     previousRobot = selectedrobot
        # else:
        	# if t%100==0 or flag==True:
    	showmap.root.after(1,showmap.updateRobotMap,robots[selectedrobot],selectedrobot,world.posbyrobots.values())

        #if t%100==0 or flag==True:
    	showmap.root.after(1,showmap.updateGraphics,world.worldmap,world.posbyrobots[robots[selectedrobot]],selectedrobot,t)
        # for robot in world.posbyrobots.keys():
            # print robot.currentPercept
        # raw_input("asd")
    # gc.collect()
    showmap.root.after(10,run,t)



listofbehaviours=[robot.Robot.gradientmove, robot.Robot.randomMove]
selectedrobot = 0
previousRobot = 0
worldmap=readmap()
world=world.World(np.array(worldmap), [(2, 2),(1,1),(3,3),(2, 2),(1,1),(3,3),(2, 2),
                (1,1),(3,3),(2, 2),(1,1),(3,3),(2, 2),(1,1),(3,3),(2, 2),(1,1),(3,3)])
robots =  world.posbyrobots.keys()
for robot in robots:
    rs, robot.currentPercept = world.getsubmap(robot)
    robot.expandperceptmap()
    # print robot.goalList

showmap=graphics.Graphics(len(worldmap))
for i in range(0,len(world.robotsbypos)):
    showmap.listbox.insert(tk.END,str(i))

showmap.listbox.activate(0)
flag=False
showmap.creategraphics(world.worldmap, world.posbyrobots.keys()[selectedrobot], selectedrobot)
showmap.root.after(2,showmap.updateGraphics,world.worldmap,world.posbyrobots.keys()[selectedrobot],world.posbyrobots[world.posbyrobots.keys()[selectedrobot]],selectedrobot,0)
#showmap.root.after(2,showmap.updateRobotMap,world.posbyrobots.keys()[selectedrobot],selectedrobot)

# raw_input("tst")
showmap.root.after(1,run, 0)
# i = 0
# while(True):
#     run(i)
#     i+=1
showmap.root.mainloop()
