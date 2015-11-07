import numpy as np
import random
from Tkinter import *
import time

size =400
worldmap = [[random.randint(0,2) for i in range(size)] for i in range(size)]
robotmap = [[random.randint(-1,2) for i in range(2*size)] for i in range(2*size)]
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

def graphMap(worldmap):
    size = len(worldmap)
    blocksize = (height-5)/float(size)
    for i in range(size):
        for j in range(size):
            color = "white"
            if worldmap[i][j] == 1:
                color = "black"
            if worldmap[i][j] == 2:
                color = "red"
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
    size = len(robotmap)
    blocksize = (height-5)/float(size/2)
    for i in range(size):
        for j in range(size):
            color = "white"
            if robotmap[i][j] == -1:
                color = "gray"
            if robotmap[i][j] == 1:
                color = "black"
            if robotmap[i][j] == 2:
                color = "red"
            rmap[i,j] = canvas.create_rectangle(i*blocksize+height+5,j*blocksize+5,i*blocksize+blocksize+height+5,j*blocksize+blocksize+5, outline=color, fill=color)
    robotlabel.pack()
    canvas.create_window(height/2, height + 60, window=robotlabel)
    robotidtext.set('Robot: ' + str(robotid))

def updateMap(worldmap):
    size = len(worldmap)
    blocksize = (height-5)/float(size)
    for i in range(size):
        for j in range(size):
            color = "white"
            if worldmap[i][j] == 1:
                color = "black"
            if worldmap[i][j] == 2:
                color = "red"
            canvas.itemconfig(rect[i,j], outline=color, fill=color)

def updateRobotMap(robotmap,robotid):
    size = len(robotmap)
    blocksize = (height-5)/float(size/2)
    for i in range(size):
        for j in range(size):
            color = "white"
            if robotmap[i][j] == -1:
                color = "gray"
            if robotmap[i][j] == 1:
                color = "black"
            if robotmap[i][j] == 2:
                color = "red"
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
        worldmap = [[random.randint(0,2) for i in range(size)] for i in range(size)]
        root.after(10000, run, t,worldmap,robotmap,robotid)
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

        
