import numpy as np
import random
import Tkinter as tk
import time
import utils
class Graphics(object):
    def __init__(self,size):
        self.size = size
        self.rsize = size*2
        self.root = tk.Tk()
        self.height = 300
        self.canvas = tk.Canvas(self.root, width=3*self.height, height = 2*self.height)
        self.canvas.pack()
        self.timetext = tk.StringVar()
        self.timelabel = tk.Label(self.canvas, textvariable=self.timetext, fg='black', bg='white')
        self.robotidtext = tk.StringVar()
        self.robotlabel = tk.Label(self.canvas, textvariable=self.robotidtext, fg='black', bg='white')
        self.listbox = tk.Listbox(self.canvas)
        self.rect = {}
        self.rmap={}
        self.colors = {utils.MAPREP.EMPTY:"white",utils.MAPREP.BLOCKED:"black",utils.MAPREP.SELF:"red",utils.MAPREP.PEER:"blue",utils.MAPREP.UNEXPLORED:"gray"}
    
    def graphMap(self, worldmap):
        blocksize = (self.height-5)/float(self.size)
        for i in range(self.size):
            for j in range(self.size):
                color =  self.colors[worldmap[i,j]]
                self.rect[i,j]=(self.canvas.create_rectangle(i*blocksize+5,j*blocksize+5,i*blocksize+blocksize+5,j*blocksize+blocksize+5, outline=color, fill=color))
        self.timelabel.pack()
        self.canvas.create_window(self.height/2, self.height + 20, window=self.timelabel)
        self.listbox.pack()
        self.canvas.create_window(self.height/2, self.height+160,window = self.listbox)
        self.timetext.set('Time: 0')

    def graphRobotMap(self, robot,robotid):
        robotmap = robot.perceptmap
        self.canvas.create_line(self.height+1,0,self.height,2*self.height, width = 5)
        blocksize = (self.height-5)/float(self.size)
        for i in range(self.rsize):
            for j in range(self.rsize):
                color = self.colors[robotmap[i,j]]
                self.rmap[i,j] = self.canvas.create_rectangle(i*blocksize+self.height+5,j*blocksize+5,i*blocksize+blocksize+self.height+5,j*blocksize+blocksize+5, outline=color, fill=color)
        self.robotlabel.pack()
        self.canvas.create_window(self.height/2, self.height + 60, window=self.robotlabel)
        self.robotidtext.set('Robot: ' + str(robotid))

    def updateMap(self, worldmap,selrobotpos):
        blocksize = (self.height-5)/float(self.size)
        for i in range(self.size):
            for j in range(self.size):
                color =self.colors[worldmap[i,j]]
                self.canvas.itemconfig(self.rect[i,j], outline=color, fill=color)
        color = self.colors[utils.MAPREP.SELF]
        self.canvas.itemconfig(self.rect[selrobotpos],outline = color,fill = color)

    def updateRobotMap(self, robot,robotid):
        blocksize = (self.height-5)/float(self.size/2)
        robotmap = robot.perceptmap
        for i in range(robot.minxposition,robot.maxxposition+1):
            for j in range(robot.minyposition,robot.maxyposition+1):
                color = self.colors[robotmap[i,j]]
                self.canvas.itemconfig(self.rmap[i,j], outline=color, fill=color)
        self.robotidtext.set('Robot: ' + str(robotid))

    def creategraphics(self, worldmap,robotmap,robotid):
        self.graphMap(worldmap)
        self.graphRobotMap(robotmap,robotid)

    def updateGraphics(self, worldmap,robotmap,selrobotpos,robotid,t):
        self.updateMap(worldmap,selrobotpos)
        #self.updateRobotMap(robotmap,robotid)
        self.timetext.set('Time: ' + str(t))
