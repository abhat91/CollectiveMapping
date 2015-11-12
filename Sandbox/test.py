import numpy as np
import random
from Tkinter import *
import time

size =400
rsize = size*2
worldmap = np.random.randint(4, size=(size,size))#[[random.randint(0,2) for i in range(size)] for i in range(size)]
worldmap = worldmap-1
robotmap = np.zeros((rsize,rsize))#[[-1 for i in range(2*size)] for i in range(2*size)]
robotmap = robotmap-1
rmap = {}
root = Tk()
height =300
canvas = Canvas(root, width=3*height, height = 2*height)
canvas.pack()
timetext = StringVar()
timelabel = Label(canvas, textvariable=timetext, fg='black', bg='white')
robotidtext = StringVar()
robotlabel = Label(canvas, textvariable=robotidtext, fg='black', bg='white')
rect = {}
time = 0

colors = {0:"white",1:"black",2:"red",-1:"gray"}

def graphMap(worldmap):
    #size = len(worldmap)
    blocksize = (height-5)/float(size)
    for i in range(size):
        for j in range(size):
            color =  colors[worldmap[i,j]]
            rect[i,j]=(canvas.create_rectangle(i*blocksize+5,j*blocksize+5,i*blocksize+blocksize+5,j*blocksize+blocksize+5, outline=color, fill=color))
    timelabel.pack()
    canvas.create_window(height/2, height + 20, window=timelabel)
    timetext.set('Time: 0')

'''def graphRobots(worldmap):
    for i in robotgraphics:
        canvas.delete(i)
    size = len(worldmap)
    blocksize = (height-5)/float(size)
    for i in range(size):
        for j in range(size):
            if worldmap[i][j] == 2:
                robot = canvas.create_oval(i*blocksize+5,j*blocksize+5,i*blocksize+blocksize+5,j*blocksize+blocksize+5, outline="red", fill="red")
                robotgraphics.append(robot)'''
                
def graphRobotMap(robotmap,robotid):
    canvas.create_line(height+1,0,height,2*height, width = 5)
    blocksize = (height-5)/float(size/2)
    for i in range(rsize):
        for j in range(rsize):
            color = colors[robotmap[i,j]]
            rmap[i,j] = canvas.create_rectangle(i*blocksize+height+5,j*blocksize+5,i*blocksize+blocksize+height+5,j*blocksize+blocksize+5, outline=color, fill=color)
    robotlabel.pack()
    canvas.create_window(height/2, height + 60, window=robotlabel)
    robotidtext.set('Robot: ' + str(robotid))

def updateMap(worldmap):
    blocksize = (height-5)/float(size)
    for i in range(size):
        for j in range(size):
            color =colors[worldmap[i,j]]
            canvas.itemconfig(rect[i,j], outline=color, fill=color)

def updateRobotMap(robotmap,robotid):
    blocksize = (height-5)/float(size/2)
    for i in range(rsize):
        for j in range(rsize):
            color = colors[robotmap[i,j]]
            canvas.itemconfig(rmap[i,j], outline=color, fill=color)
    robotidtext.set('Robot: ' + str(robotid))
    
def creategraphics(worldmap,robotmap,robotid):
    graphMap(worldmap)
    graphRobotMap(robotmap,robotid)

def updateGraphics(worldmap,robotmap,robotid):
    updateMap(worldmap)
    updateRobotMap(robotmap,robotid)
    
def run(t,worldmap,robotmap,robotid):
    if t < 100:
        t = t + 1
        timetext.set('Time: ' + str(t))
        worldmap = np.random.randint(4, size=(size,size)) -1
        root.after(1000, run, t,worldmap,robotmap,robotid)
        root.after(1,updateGraphics,worldmap,robotmap,robotid)

#graphMap(worldmap)
#graphRobots(worldmap)
#graphRobotMap(robotmap)
robotid = 1
creategraphics(worldmap,robotmap,robotid)
t = 0
root.after(1000,run, t,worldmap,robotmap,robotid)
root.mainloop()
#root.mainloop()

        
