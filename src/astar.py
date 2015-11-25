import math
import Queue
import utils
import heapq

direction = {
0 : utils.MOVES.NORTH,
1 : utils.MOVES.NORTHEAST,
2 : utils.MOVES.EAST,
3 : utils.MOVES.SOUTHEAST,
4 : utils.MOVES.SOUTH,
5 : utils.MOVES.SOUTHWEST,
6 : utils.MOVES.WEST,
7 : utils.MOVES.NORTHWEST
}

def calcdistance(point,goal):
    dx = abs(point[0]-goal[0])
    dy = abs(point[1]-goal[1])
    dist = 1*(dx + dy)
    #dist = 2*math.sqrt(pow(dx,2)+pow(dy,2))
    return dist

class Node(object):
    def __init__(self,loc):
        self.loc = loc
        self.f = 0
        self.length = 0
        self.path = []

    def __cmp__(self,other):
        return cmp(self.f,other.f)

class PathFinder(object):
    def __init__(self,world,size,start,goal):
        self.world = world
        self.size = size
        self.start = start
        self.goal = goal
        self.nodes = []
    
    def getPossibleMoves(self,loc):
        possiblemoves = []
        for i in direction.keys():
            dir = direction[i]
            dx = dir[0]
            dy = dir[1]
            if self.world[loc[0]+dx,loc[1]+dy] != utils.MAPREP.BLOCKED:#direction it wants to move is empty
                if abs(dx) == 1 and abs(dy) == 1:#if direction is diagonal
                    if self.world[loc[0]+dx,loc[1]] == utils.MAPREP.EMPTY and self.world[loc[0],loc[1]+dy] == utils.MAPREP.EMPTY:#if diagonal is clear
                        possiblemoves.append((dx,dy))
                else:#if it's not a diagonal
                    possiblemoves.append((dx,dy))
        return possiblemoves

    def getNeighbors(self,node):
        length = node.length + 1
        loc = node.loc
        for i in self.getPossibleMoves(loc):
            if (loc[0]+i[0],loc[1]+i[1])==self.goal:
                return node
            path = [k for k in node.path]
            if self.world[loc[0]+i[0],loc[1]+i[1]]!=utils.MAPREP.UNEXPLORED and (loc[0]+i[0],loc[1]+i[1]) not in path:
                dist = calcdistance((loc[0]+i[0],loc[1]+i[1]),self.goal)
                path.append((loc[0]+i[0],loc[1]+i[1]))
                n = Node((loc[0]+i[0],loc[1]+i[1]))
                n.path = path
                n.length = length
                n.f = length+dist
                heapq.heappush(self.nodes,(n.f,n))
        return False
        
    def run(self):
            start = Node(self.start)
            start.f = calcdistance(self.start, self.goal)
            heapq.heappush(self.nodes,(start.f,start))
            while 1:
                if len(self.nodes) ==0:
                    print "NO VALID PATH"
                    return [(0,0)]
                n = heapq.heappop(self.nodes)[1]
                sol = self.getNeighbors(n)
                if isinstance(sol,Node):
                    return sol.path

