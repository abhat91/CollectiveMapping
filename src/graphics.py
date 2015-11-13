import numpy as np
import random
from Tkinter import*
import time

class Graphics(object):
    def __init__(self,size):
        self.size = size
        self.rsize = size*2
        self.root = tk()
        self.height = 300
        self.canvas = Canvas(root, width=3*height, height = 2*height)
        self.canvas.pack()
        self.timetext = StringVar()
        self.timelabel = Label(canvas, textvariable=timetext, fg='black', bg='white')
        self.robotidtext = StringVar()
        self.robotlabel = Label(canvas, textvariable=robotidtext, fg='black', bg='white')
        self.rect = {}
        self.rmap={}
        self.colors = {3:"white",4:"black",2:"red",1:"blue",0:"gray"}

    def graphMap(worldmap):
        blocksize = (self.height-5)/float(self.size)
        for i in range(self.size):
            for j in range(self.size):
                color =  self.colors[worldmap[i,j]]
                self.rect[i,j]=(self.canvas.create_rectangle(i*blocksize+5,j*blocksize+5,i*blocksize+blocksize+5,j*blocksize+blocksize+5, outline=color, fill=color))
        self.timelabel.pack()
        self.canvas.create_window(self.height/2, self.height + 20, window=self.timelabel)
        self.timetext.set('Time: 0')

    def graphRobotMap(robotmap,robotid):
        self.canvas.create_line(self.height+1,0,self.height,2*self.height, width = 5)
        blocksize = (self.height-5)/float(self.size/2)
        for i in range(self.rsize):
            for j in range(self.rsize):
                color = self.colors[robotmap[i,j]]
                self.rmap[i,j] = self.canvas.create_rectangle(i*blocksize+self.height+5,j*blocksize+5,i*blocksize+blocksize+self.height+5,j*blocksize+blocksize+5, outline=color, fill=color)
        robotlabel.pack()
        canvas.create_window(height/2, height + 60, window=robotlabel)
        robotidtext.set('Robot: ' + str(robotid))
        
    def updateMap(worldmap):
        blocksize = (self.height-5)/float(self.size)
        for i in range(self.size):
            for j in range(self.size):
                color =self.colors[worldmap[i,j]]
                self.canvas.itemconfig(self.rect[i,j], outline=color, fill=color)

    def updateRobotMap(robotmap,robotid):
        blocksize = (self.height-5)/float(self.size/2)
        for i in range(self.rsize):
            for j in range(self.rsize):
                color = self.colors[robotmap[i,j]]
                self.canvas.itemconfig(self.rmap[i,j], outline=color, fill=color)
        self.robotidtext.set('Robot: ' + str(robotid))
        
    def creategraphics(worldmap,robotmap,robotid):
        graphMap(worldmap)
        graphRobotMap(robotmap,robotid)

    def updateGraphics(worldmap,robotmap,robotid,t):
        updateMap(worldmap)
        updateRobotMap(robotmap,robotid)
        self.timetext.set('Time: ' + str(t))

    
