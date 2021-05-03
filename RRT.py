import random
import math
import pygame

class Map:
    def __init__(self,startX,startY,goalX,goalY,w,h,obsDims,obsNum):
        self.startX = startX
        self.startY = startY
        self.goalX = goalX
        self.goalY = goalY
        self.w = w
        self.h = h

        #obstacles 
        self.obsDims = obsDims
        self.obsNum = obsNum
        self.obstacles = []

        # setting the window
        pygame.display.set_caption('RRT Path Planning')
        self.map = pygame.display.set_mode((self.w, self.h))
        self.map.fill((255, 255, 255))
        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 1

        #defining different colors
        self.grey = (70, 70, 70)
        self.Blue = (0, 0, 255)
        self.Green = (0, 255, 0)
        self.Red = (255, 0, 0)
        self.white = (255, 255, 255)


    def drawArena(self, obstacles):
        pygame.draw.circle(self.map, self.Green, (self.startX,self.startY), self.nodeRad + 5, 0)
        pygame.draw.circle(self.map, self.Red, (self.goalX,self.goalY), self.nodeRad + 20, 1)
        self.addObs(obstacles)

    def addLine(self, path):
        for node in path:
            pygame.draw.circle(self.map, self.Red, node, 3, 0)

    def addObs(self, obstacles):
        obsList = obstacles.copy()
        while (len(obsList) > 0):
            obstacle = obsList.pop(0)
            pygame.draw.rect(self.map, self.grey, obstacle)


class RRT:
    def __init__(self,startX,startY,goalX,goalY,w,h,obsDims,obsNum):
        self.startX = startX
        self.startY = startY
        self.goalX = goalX
        self.goalY = goalY
        self.w = w
        self.h = h
        self.goalReached = False

        #obstacles
        self.obsDims = obsDims
        self.obsNum = obsNum
        self.obstacles = []

        #initializing the tree
        self.x = []
        self.y = []
        self.parent = []
        self.x.append(startX)
        self.y.append(startY)
        self.parent.append(0)

        #for path
        self.goalState = None
        self.path = []


    def makeobs(self):
        obs = []
        
        for i in range(0, self.obsNum):
            rectangle = None
            startgoalpoint = True
            while startgoalpoint:
                obx = int(random.uniform(0, self.w- self.obsDims))
                oby = int(random.uniform(0, self.h - self.obsDims))
                rectangle = pygame.Rect((obx,oby), (self.obsDims, self.obsDims))
                if (rectangle.collidepoint(self.startX,self.startY) or rectangle.collidepoint(self.goalX,self.goalY)):
                        startgoalpoint = True
                else:
                    startgoalpoint = False
            obs.append(rectangle)
        self.obstacles = obs.copy()
        return obs

    # if no collision found then add node
    def addNode(self,n,x,y): 
        self.x.insert(n,x)
        self.y.insert(n,y)

    #if the node is in collision then remove node
    def removeNode(self,n):
        self.x.pop(n)
        self.y.pop(n)

    #if there is no collison then add edge
    def addEdge(self,parent,child): 
        self.parent.insert(child,parent)

    #if collision occurs then remove edge
    def removeEdge(self,n): 
         self.parent.pop(n)

    #to get number of nodes
    def numberOfNodes(self): 
        return len(self.x)

    #to calculate the distace between two points
    def distance(self,n1,n2): 
        (x1,y1) = (self.x[n1],self.y[n1])
        (x2,y2) = (self.x[n2],self.y[n2])
        px = (float(x1)-float(x2))**2
        py = (float(y1)-float(y2))**2

        return (px+py)**0.5

    #to get the nearest point in space
    def nearest(self,n):
        dmin = self.distance(0,n)
        nnear = 0 
        for i in range (0,n):
            if self.distance(i,n) < dmin:
                dmin = self.distance(i,n)
                nnear = i
        return nnear

    #to generate random co-ordinates in the arena
    def randomPoints(self): #to generate a random variable in availabe space
        rx = int(random.uniform(0,self.w))
        ry = int(random.uniform(0,self.h))

        return rx,ry

    #to check if the point is in free space or not 
    def freeSpace(self): 
        n = self.numberOfNodes() - 1
        (x, y) = (self.x[n], self.y[n])
        obs = self.obstacles.copy()
        while len(obs) > 0:
            obstacle = obs.pop(0)
            if obstacle.collidepoint(x, y):
                self.removeNode(n)
                return False
        return True

    #to check there are no collisions with obstacles 
    def collision(self, x1, x2, y1, y2): 
        obs = self.obstacles.copy()
    
        while (len(obs) >0) :
            obstacle = obs.pop(0)
            for i in range(0,301):
                u = i/300
                x= x1*u + x2*(1-u)
                y= y1*u + y2*(1-u)
                if obstacle.collidepoint(x, y):
                    return True
        return False

    #to connect nodes
    def line(self, n1, n2):
        x1 = self.x[n1]
        x2 = self.x[n2]
        y1 = self.x[n1]
        y2 = self.x[n2]

        if self.collision(x1, x2, y1, y2):
            self.removeNode(n2)
            return False
        else:
            self.addEdge(n1,n2)
            return True

    #to check the random co-ordinate is near to which of the nodes
    def step(self, nnear, nrand, dmax=35):
        d = self.distance(nnear, nrand)
        if d > dmax:
            u = dmax / d
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            (px, py) = (xrand - xnear, yrand - ynear)
            theta = math.atan2(py, px)
            (x, y) = (int(xnear + dmax * math.cos(theta)),
                      int(ynear + dmax * math.sin(theta)))
            self.removeNode(nrand)
            if abs(x - self.goalX) <= dmax and abs(y - self.goalY) <= dmax:
                self.addNode(nrand, self.goalX, self.goalY)
                self.goalState = nrand
                self.goalReached = True
            else:
                self.addNode(nrand, x, y)

    #to create nodes that will go towards the goal
    def bias(self, ngoalX, ngoalY):
        n = self.numberOfNodes()
        self.addNode(n, ngoalX, ngoalY)
        nnear = self.nearest(n)
        self.step(nnear, n)
        self.line(nnear, n)
        return self.x, self.y, self.parent

    #to generate random co-ordinates in the free space available 
    def expand(self):
        n = self.numberOfNodes()
        x, y = self.randomPoints()
        self.addNode(n, x, y)
        if self.freeSpace():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            self.line(xnearest, n)
        return self.x, self.y, self.parent

    #to save nodes which lead to goal state from start state
    def goalPath(self):
        if self.goalReached:
            self.path = []
            self.path.append(self.goalState)
            newpos = self.parent[self.goalState]
            while (newpos != 0):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goalReached

    #to extract x,y co-ordinates to plot the end path formed
    def pathcoords(self):
        pathCoords = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            pathCoords.append((x, y))
        return pathCoords


def main():
    startX= 50
    startY = 50
    goalX = 300
    goalY = 300
    obsNum = 10
    obsDims = 20
    w = 500
    h = 500
    iterations = 0

    pygame.init()
    map = Map(startX,startY,goalX,goalY,w,h,obsDims,obsNum)
    output = RRT(startX,startY,goalX,goalY,w,h,obsDims,obsNum)

    obs = output.makeobs()
    map.drawArena(obs)
    

    while (not output.goalPath()):
        if iterations % 10 == 0:
            X, Y, Parent = output.bias(goalX,goalY)
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad+2, 0)
            pygame.draw.line(map.map, map.Blue, (X[-1], Y[-1]), (X[Parent[-1]],Y[Parent[-1]]), map.edgeThickness)
        else:
            X, Y, Parent = output.expand()
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad+2, 0)
            pygame.draw.line(map.map, map.Blue, (X[-1], Y[-1]), (X[Parent[-1]],Y[Parent[-1]]), map.edgeThickness)

        if iterations % 5 == 0:
            pygame.display.update()
        iterations += 1

    map.addLine(output.pathcoords())

    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)

if __name__ == '__main__':
    main()
