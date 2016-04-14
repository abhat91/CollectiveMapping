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
import time
import sys

def convertToInteger(multidimentionalList):
    """Converts to integer"""
    return map(int, multidimentionalList)

def readmap(folder,f):
    """Method to read the map"""
    data = [line.rstrip('\n') for line in open(folder+ f)]
    if len(data)==1:
    	data[0]=data[0].replace('\t', ' ')
    	data=[line for line in data[0].split('\r')]
    data = [ map(str,line.split(' ')) for line in data ]
    return map(convertToInteger, data)



mapfolder = os.getcwd()+"/../worlds/"
maps = [["sparsermaze24_24.txt","sparsermaze50_50.txt","sparsermaze100_100.txt","sparsermaze200_200.txt"],
        ["sparsemaze24_24.txt","sparsemaze48_48.txt","sparsemaze100_100.txt","sparsemaze200_200.txt"],
        ["maze24_24.txt","maze50_50.txt","maze100_100.txt","maze200_200.txt"]]
TIME_LIMIT = 40000
map_size=  ["small","medium","large","extra-large"]
map_density = ["sparse","moderate","dense"]
algorithms = ["random","Markov-Walk","Gradient","Targeted MIG","Greedy MIG"]
print "time step;map Size;Density;Algorithm;Number of Robots;Percent complete;simulation time"

for mapsize in range(len(map_size)):
    for mapdensity in range(len(map_density)):
        for alg in range(len(algorithms)):
            for nRobots in range(1,25):
                # print maps[mapdensity][mapsize]
                worldmap = np.array(readmap(mapfolder,maps[mapdensity][mapsize]));
                robotlocations = np.array(np.random.random_sample((nRobots,2))*(len(worldmap)-2) + 1,np.uint8).tolist()
                # print robotlocations
                maparea = len(worldmap)**2
                w = world.World(worldmap,robotlocations,alg)
                robots =  w.posbyrobots.keys()
                start_time = time.clock()
                for t in range(TIME_LIMIT):
                    for robot in robots:
                        if robot.robotStep() == "Explored":
                            end_time = time.clock()
                            explored = float(np.count_nonzero(robot.perceptmap))/maparea*100
                            print t,";",map_size[mapsize],";",map_density[mapdensity],";",algorithms[alg],";",nRobots,";",str(explored)+"% ; ",(end_time - start_time)
                            break
                        else:
                            end_time = time.clock()
                            explored=[]
                            for robot in robots:
                                explored=explored+[np.count_nonzero(robot.perceptmap)]
                        	maxexplored=float(heapq.nlargest(1, explored)[0])/maparea*100
                            print t,";",map_size[mapsize],";",map_density[mapdensity],";",algorithms[alg],";",nRobots,";",str(maxexplored)+"% ; ",(end_time - start_time)

                    else:
                        continue # executed if the loop ended normally (no break)
                    break # executed if 'continue' was skipped (break)
