import numpy as np
import Queue
import utils
class Astar2:
    def h(self,node):
        #print node
        #print self.destination
        return abs(node[0]-self.destination[0])**2 + abs(node[1]-self.destination[1])
    def __init__(self,perceptmap,currentpercept,current,destination):
        self.perceptmap = perceptmap
        self.radius = (len(currentpercept)-1)/2
        #update it's own perceptmap with the current percept for nearby robots
        self.perceptmap[current[0]-self.radius:current[0]+self.radius+1,
                        current[1]-self.radius:current[1]+self.radius+1] = currentpercept
        self.destination = destination[1]
        self.frontier = Queue.Queue()
        self.current=  current
        self.frontier.put(self.current,0)
        self.came_from = {}
        self.cost = {}
        self.came_from[self.current] = None
        self.cost[self.current]= 0
    def neighbors(self,parent):
        neighbors = []
        #print range(-self.radius,self.radius+1,1)
        for i in range(-self.radius,self.radius+1,1):
            for j in range(-self.radius,self.radius+1,1):
                if not(i == 0 and j == 0):
                    node = (parent[0]+i,parent[1]+j)
                    #print node
                    if self.perceptmap[node] in [utils.MAPREP.EMPTY,utils.MAPREP.UNEXPLORED] :
                        #print 'empty'
                        if abs(i) == 1 and abs(j) == 1:
                            #print 'diag'
                            if self.perceptmap[parent[0]+i,parent[1]] in [utils.MAPREP.EMPTY,utils.MAPREP.UNEXPLORED] and self.perceptmap[parent[0],parent[1]+j] in [utils.MAPREP.EMPTY,utils.MAPREP.UNEXPLORED]:
                                #print 'appended'
                                neighbors.append(node)
                        else:
                            #print 'appended'
                            neighbors.append(node)
        #print neighbors
        return neighbors


    def search(self):
        while not self.frontier.empty():
            #print self.frontier
            current = self.frontier.get()
            #print current
            if current == self.destination:
                print 'got it', self.getPath()
                return self.getPath()
            for nextmove in self.neighbors(current):
                #print nextmove,self.came_from
                nextcost = self.cost[current] + 1
                if nextmove not in self.cost or nextcost < self.cost[nextmove]:
                    self.cost[nextmove] = nextcost
                    priority = nextcost + self.h(nextmove)
                    self.frontier.put(nextmove,priority)
                    self.came_from[nextmove] = current
        print 'ops',self.current, self.destination
        return False


    def getPath(self):
        current = self.destination
        path = []
        while current != self.current:
            path.append(current)
            current = self.came_from[current]
        path.reverse()
        return path
