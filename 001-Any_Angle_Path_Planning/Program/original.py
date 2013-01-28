# Authors: Tina Nye & Qandeel Sajid
# Project: AI-Project 1

#! /usr/bin/env python
# Pygame visualization template

import pygame
import math
import heapq
import sys
import time


import random
from apgl.graph.SparseGraph import *
from apgl.graph.VertexList import *
import numpy
import scipy.sparse

###### Print Intro
print "\nProgram: Any-Angle Path Planning"
print "By: Tina Nye & Qandeel Sajid"

##############################################################################
#-----CONSTANTS-------------------------------------------------
# Define some colors
black = ( 0, 0, 0)
white = ( 255, 255, 255)
green = ( 0, 255, 0)
blue=(0,0,255)
red = ( 255, 0, 0)
gray=(96, 96, 96)


percentBlocked=.1  #--10% 

# This sets the width and height of each grid location
width=11
height=11

# This sets the margin between each cell
margin=1

# Set the row and column length
rowLength = 52
colLength = 102
file_provided=False
# Create a 2 dimensional array. A two dimesional array is simply a list of lists.
grid=[]
for row in range(rowLength):
   # Add an empty array that will hold each cell in this row
   grid.append([])
   for column in range(colLength):
      grid[row].append(0) # Append a cell
#-----------------------------------------------------------------

#############################################################################
##############################################################################
#check if file is provided
#if so, get obstacles, start/goal from file

try:
    fin=open(sys.argv[1], "r")
except IOError:
    print '\nERROR: Cannot open file, will use random obstacles and start/goal'
else:
    obstacles=[]
    print sys.argv[1]
    file_provided=True
    
    #read every thing from file
    fileInfo=[]
    fileInfo.append(fin.readline())
    while fileInfo[len(fileInfo)-1]:
        fileInfo.append(fin.readline())
    
    # Get obstacles from file
    for i in range (len(fileInfo)-3):
        val= (fileInfo[i].split(' '))
        #print str(i) +":"+str(val) + " " + str(val[0])+ " " + str(val[1])
        x=int(val[0])
        y=int(val[1])
        #print str(x)+ " "+str(y)
        obstacles.append([x,y])  #get the x, y coordinated
    
    #get start/goal from end of file
    val= (fileInfo[len(fileInfo)-3].split(' '))
    x=int(val[0])
    y=int(val[1])
    start=[x,y]
    
    val= fileInfo[len(fileInfo)-2].split(' ')
    x=int(val[0])
    y=int(val[1])
    goal=[x,y]

##############################################################################
##############################################################################
#########################################################################
#---- Common Functions ---------------------------------------
#this just uses the x/y value of a vertex to give the distance to goal (heuristic)
def getDistanceToGoal(x,y):
    return math.sqrt(math.pow((x-goal[0]),2)+math.pow((y-goal[1]),2))

def getDistance(x,y, x1, y1):
    return math.sqrt(math.pow((x-x1),2)+math.pow((y-y1),2))

# This function takes in teh x value and y value of teh vertex and gives the vertex id
def getVertexID(x, y):
    return (((colLength-1)*(y-1))+(x-1))

#for converting x,y to pixel and back
def convertRowToPixel(s):
    solution=(margin+height)*s
    return solution

def convertColToPixel(s):
    solution=(margin+width)*s
    return solution

def convertPixelToRow(s):
    solution=float(s)/(margin+height)
    return solution

def convertPixelToCol(s):
    solution=float(s)/(margin+width)
    return solution

#for removing elements from the heapqueue
def removeFromHeap(origQueue, value):
    newQueue=[]
    while(len(origQueue)!=0):
        item=heapq.heappop(origQueue)
        if(value!=item[2]):
            heapq.heappush(newQueue, item)
    return newQueue

#########################################################################
################################################################################
################################################################################
##############################################################################
#--- Randomly pick out 10% BLOCKED CELLS ---
#This sections first randomly picks random vertices and makes them into obstacles. The if statement ensures that the border does not count as the 10% of blocked vertices
if(file_provided==False):
    obstacles=[]
    maxBlockCells=int(math.floor((rowLength-2)*(colLength-2)*percentBlocked))
    #print "number of blocked cells: " + str(maxBlockCells)

    while(len(obstacles)!=maxBlockCells):
        x=int(math.floor(random.random()*(colLength-1)))
        y=int(math.floor(random.random()*(rowLength-1)))
        #print str(x) +", y="+ str(y)
        if(obstacles.count([x,y])==0 and x!=0 and x!=colLength and y!=0 and y!=rowLength):
            obstacles.append([x,y])

    # this part blocks out the bordering cells
    for i in range(colLength):
        obstacles.append([i,0])
        obstacles.append([i, rowLength-1])

    for i in range(rowLength):
        obstacles.append([0, i])
        obstacles.append([colLength-1,i])

#############################################################################
################################################################################
################################################################################
#############################################################################
#---- Making the Graph ---------------------------------------------
# This initializes the graph, just basic things (its in the SparseGraph Ex)
numFeatures=0
numVertices=(rowLength-1)*(colLength-1)
vList=GeneralVertexList(numVertices)
weightMatrix = scipy.sparse.lil_matrix((numVertices, numVertices))
graph = SparseGraph(vList,True, W=weightMatrix)  # HERE IS YOUR GRAPH!


#-- This assigns to each vertex an array as a value. In the array, the first value is the x location, and then y (row), and 1 implies the vertex is unexplored. 0 will mean it is closed. The fourth value determines whether the vertex is in the fringe (1) or not (0)
#-- 5th value-g(A*), h(A*), f(A*), g(Theta*), h(Theta*), f(Theta*)
row=1
for i in range (graph.getNumVertices()):
    if(i%(colLength-1)==0 and i!=0):
        row=row+1
    graph.setVertex(i, [(i%(colLength-1))+1,row, 1, 0,100000000000.0,0.0, 100000000000.0,100000000000.0, 0.0, 100000000000.0])

""" check the vertex values
for i in range (graph.getNumVertices()):
    print "vertexVal: "+str(i)+ " " + str(graph.getVertex(i))
"""

#----- Connect vertices
# This just runs through and connects the vertices
# This is pretty complicated, and you don't need to worry about it. The code here should be right becuase the red edges are draen correctly
edgeCheck=[]
for row in range((rowLength-1)):
    for column in range(colLength-1):
        x=column+1
        y=row+1
        vertex1=(((colLength-1)*row)+column)
        vertex2=(((colLength-1)*row)+column)+1

        if((((colLength-1)*row)+column)<numVertices and edgeCheck.count([vertex2,vertex1])==0 and vertex1%(colLength-1)<(colLength-2)):
            if(not(obstacles.count([x,y])!=0 and obstacles.count([x,(y-1)])!=0)):
                #print "v1: " + str(vertex1) + "v2: " + str(vertex2)
                graph.addEdge(vertex1,vertex2, edge=1)


        vertex2=(((colLength-1)*row)+column)+(colLength-1)
        if((((colLength-1)*row)+column)<numVertices and edgeCheck.count([vertex2,vertex1])==0 and vertex2<numVertices):
            if(not(obstacles.count([x,y])!=0 and obstacles.count([(x-1),y])!=0)):
                #print "v1: " + str(vertex1) + "v2: " + str(vertex2)
                graph.addEdge(vertex1,vertex2, edge=1)

        #do the diagonals
        vertex2=(((colLength-1)*row)+column)+(colLength)
        if(obstacles.count([x,y])==0 and (((colLength-1)*row)+column)<numVertices and edgeCheck.count([vertex2,vertex1])==0 and vertex1%(colLength-1)<(colLength-1) and vertex1%(rowLength-1)<(rowLength-1) and vertex2<numVertices and vertex1%(colLength-1)!=(colLength-2)):
            graph.addEdge(vertex1,vertex2, edge=math.sqrt(2))

        vertex1=(((colLength-1)*row)+column)
        vertex2=(((colLength-1)*row)+column)+(colLength-2)
        if((((colLength-1)*row)+column)<numVertices and vertex1%(colLength-1)<(colLength-1) and vertex1%(rowLength-1)<(rowLength-1) and vertex2<numVertices and vertex1%(colLength-1)!=0):
            if(not(obstacles.count([(x-1),y])!=0)):
                graph.addEdge(vertex1,vertex2, edge=math.sqrt(2))


numEdges=graph.getNumEdges()

print "\nVertices: " + str(graph.getNumVertices())
print "Edges: " + str(graph.getNumEdges())

#-----------------------------------
# Here is where we choose a random start/goal locations. You probably don't need to touch any of this. 
if(file_provided==False):
    goal=[]
    start=[]
    #---- Get random start ------------------
    while start==[]:
        id=int((i*random.random())%i)
        vrtx=graph.getVertex(id)
        if(len(graph.neighbours(id))>1):
            x=vrtx[0]
            y=vrtx[1]
            if(x!=0 and x!=colLength and y!=0 and y!=rowLength):
                start=[x,y]  

    #---- Get random goal -------------------
    while goal==[]:
        id=int((i*random.random())%i)
        vrtx=graph.getVertex(id)
        if(len(graph.neighbours(id))>1):
            x=vrtx[0]
            y=vrtx[1]
            if(start!=[x,y] and x!=0 and x!=colLength and y!=0 and y!=rowLength):
                goal=[x,y] 

print "Start: " +str(start)
print "Goal: " +str(goal)
#--------------------------------------------------------------

################################################################################
################################################################################
################################################################################
################################################################################
#---- A* Algorithm -------------------------------------------
#-- main function for A* -------------------------------------------------
pathCost=[-1000000000 for x in range(graph.getNumVertices())] # g(n)
fringe=[] #open list!
pathFound=False

# first takes an array: first value is g+h, second is vertex id
parent=[-1 for x in range(graph.getNumVertices())]

#print "Start vertex: " + str(getVertexID(start[0], start[1]))
#print "goal vertex: " + str(getVertexID(goal[0], goal[1]))

pathCost[getVertexID(start[0], start[1])]=0 #start following algorithm
parent[getVertexID(start[0], start[1])]=getVertexID(start[0], start[1])

#push start into the queue --> heap assign an array as a value to each item. The array has the f(item), and the item's vertex id

heapq.heappush(fringe, [pathCost[getVertexID(start[0], start[1])]+getDistanceToGoal(start[0], start[1]),pathCost[getVertexID(start[0], start[1])], getVertexID(start[0], start[1])])


#mark it as "added to fringe" by changing thrid # to 1
VertexInfo=graph.getVertex(getVertexID(start[0], start[1]))
VertexInfo[3]=1
graph.setVertex(getVertexID(start[0], start[1]), VertexInfo)


#-- mini function for A*
def UpdateVertex(s, sPrime, queue):
    val=graph.getVertex(sPrime)
    if((pathCost[s]+graph.getEdge(s, sPrime))<pathCost[sPrime]):
        pathCost[sPrime]=pathCost[s]+graph.getEdge(s, sPrime)
        parent[sPrime]=s

        #remove from queue
        cpyQueue=queue
        queue=removeFromHeap(cpyQueue, sPrime)
        distance=getDistanceToGoal(val[0],val[1])
        heapq.heappush(queue, [pathCost[sPrime]+distance,pathCost[sPrime], sPrime])
        VertexInfo=graph.getVertex(sPrime)
        VertexInfo[3]=1
        VertexInfo[4]=pathCost[sPrime]
        VertexInfo[5]=distance
        VertexInfo[6]=pathCost[sPrime]+distance
        graph.setVertex(sPrime, VertexInfo)
    
    #print "cost of: " + str(sPrime) + " " + str(pathCost[sPrime]+getDistanceToGoal(val[0],val[1]))
    return queue

#start the A* loop
print "\nStarting A*"
startTimeA=time.time()
while len(fringe)!=0:
    #pop first value of queue --> checkout the beginning of "making a graph" to find out what's in the array
    vrtx=heapq.heappop(fringe)
    #print "vrtx: " + str(vrtx)
    
    #if it's goal, then break
    if(vrtx[2]==getVertexID(goal[0], goal[1])):
        print "A* found path!"
        print "Time taken: " +str(time.time()-startTimeA)
        vrtInfo=graph.getVertex(vrtx[2])
        print "Total cost: " +str(vrtInfo[4])
        pathFound=True
        
    if(pathFound==True):
        break

    #mark vertex as closed
    VertexInfo=graph.getVertex(vrtx[2])
    VertexInfo[2]=0
    graph.setVertex(vrtx[2], VertexInfo)

    #get the neighbors
    ngbrs=[]
    ngbrs=graph.neighbourOf(vrtx[2])
    #print "neighbors: " + str(ngbrs)
    #loop through neighbors
    for i in range (len(ngbrs)):
        checkNeighbor=graph.getVertex(ngbrs[i])
        if(checkNeighbor[2]==1):
            if(checkNeighbor[3]==0):
                pathCost[ngbrs[i]]=1000000000
                parent[ngbrs[i]]=-1
            cpyFringe=fringe
            fringe=UpdateVertex(vrtx[2],ngbrs[i], cpyFringe)

if(pathFound!=True):
    print "A* did not find path!"

#-------------------------------------------------------------

################################################################################
################################################################################
################################################################################
################################################################################

# --Line of Sight --------------------------------------
def lineOfSight(s, sPrime):
    #print "\nstarting line of sight"
    lineOk=True
    temp=graph.getVertex(s)
    s_x=convertColToPixel(temp[0])
    s_y=convertRowToPixel(temp[1])
    temp=graph.getVertex(sPrime)
    sPrime_x=convertColToPixel(temp[0])
    sPrime_y=convertRowToPixel(temp[1])
    #print "s's x,y location: " +str(s_x) +", " + str(s_y)
    #print "sprim's x,y location: " +str(sPrime_x) +", " + str(sPrime_y)
    if(s<sPrime):
        loc_x=float(s_x)
        loc_y=float(s_y)
        end_x=float(sPrime_x)
        end_y=float(sPrime_y)
    else: 
        loc_x=float(sPrime_x)
        loc_y=float(sPrime_y)
        end_x=float(s_x)
        end_y=float(s_y)

    if(end_x>loc_x):
        whichWay=True
    else:
        whichWay=False

    if((sPrime_y-s_y)!=0 and (sPrime_x-s_x)!=0):
        #print "slope!"
        slope=float((sPrime_y-s_y))/(float(sPrime_x-s_x))    
        b=s_y-(slope*s_x)
        #print "b: " +str(b) + " slope: " +str(slope)
        while(not(round(loc_x,10)==round(end_x,10) and round(loc_y, 10)==round(end_y, 10))):
            #print "Pix_x: "+str(loc_x) + " Pix_y: " + str(loc_y)
            #print "Pix_x: "+str(end_x) + " Pix_y: " + str(end_y)
            #print "Pix_x: "+str(round(loc_x,10)==round(end_x,10)) + " Pix_y: " + str(round(loc_y, 10)==round(end_y, 10))
            
            if(((loc_x%1.0)==0.0 and (loc_x%1.0)==0.0) and int(loc_x)==int(end_x) and int(loc_y)==int(end_y)):
                    break
                
            if(float(loc_x)<0 or float(loc_y)<0):
                print "ERROR:Pixels in negative"
                sys.exit()
            x=convertPixelToCol(loc_x)
            y=convertPixelToRow(loc_y)
            checkX=x
            checkY=y
            
            #check for blocked
            if (not((checkX%1.0)==0.0 or (checkY%1.0)==0.0) and (checkX+1)<(colLength-1) and (checkY+1)<(rowLength-1)):
                #print "x: "+str(x) + " y: " + str(y) + " not whole"
                #print str(checkX%1.0)+ " " + str(checkY%1.0)
                #print "x: "+str(math.floor(checkX)) + " y: " + str(math.floor(checkY)) + " not whole"
                if not(graph.getEdge(getVertexID(int(math.floor(checkX)), int(math.floor(checkY))), getVertexID(int(math.floor(checkX+1)), int(math.floor(checkY+1))))>0):
                    lineOk=False
        
            #update values
            if(loc_x!=end_x):
                if(whichWay):
                    loc_x=loc_x+.25
                else:
                    loc_x=loc_x-.25
                loc_y=(loc_x*slope)+b

            elif(loc_y!=end_y):
                loc_y=loc_y+.25
                loc_x=(loc_y-b)/slope
                        
    elif((sPrime_x-s_x)==0):
        #print "vertical line"
        while(not(round(loc_x,10)==round(end_x,10) and round(loc_y, 10)==round(end_y, 10))):
            #print "Pix_x: "+str(loc_x) + " Pix_y: " + str(loc_y)
            x=convertPixelToCol(loc_x)
            y=convertPixelToRow(loc_y)
            checkX=x
            checkY=y    
            
            if(((loc_x%1.0)==0.0 and (loc_x%1.0)==0.0) and int(loc_x)==int(end_x) and int(loc_y)==int(end_y)):
                break
            
            if (not((checkX%1.0)==0.0 or (checkY%1.0)==0.0) and (checkX+1)<(colLength-1) and (checkY+1)<(rowLength-1)):
                #print "x: "+str(x) + " y: " + str(y) + " not whole"
                #print str(x%1.0)+ " " + str(y%1.0)
                #print "x: "+str(math.floor(checkX)) + " y: " + str(math.floor(checkY)) + " not whole"
                if not(graph.getEdge(getVertexID(int(math.floor(checkX)), int(math.floor(checkY))), getVertexID(int(math.floor(checkX)+1), int(math.floor(checkY)+1)))>0):
                    lineOk=False
            
            #update next value
            loc_y=loc_y+.25
                                
    elif((sPrime_y-sPrime_y)==0):
        #print "horizontal line"
        while(not(round(loc_x,10)==round(end_x,10) and round(loc_y, 10)==round(end_y, 10))):
            #print "Pix_x: "+str(loc_x) + " Pix_y: " + str(loc_y)
            x=convertPixelToCol(loc_x)
            y=convertPixelToRow(loc_y)
            checkX=x
            checkY=y
            
            if(((loc_x%1.0)==0.0 and (loc_x%1.0)==0.0) and int(loc_x)==int(end_x) and int(loc_y)==int(end_y)):
                break
            
            #check for blocked
            if (not((checkX%1.0)==0.0 or (checkY%1.0)==0.0) and (checkX+1)<(colLength-1) and (checkY+1)<(rowLength-1)):
                #print "x: "+str(x) + " y: " + str(y) + " not whole"
                #print str(x%1.0)+ " " + str(y%1.0)
                #print "x: "+str(math.floor(checkX)) + " y: " + str(math.floor(checkY)) + " not whole"
                if not(graph.getEdge(getVertexID(int(math.floor(checkX)), int(math.floor(checkY))), getVertexID(int(math.floor(checkX)+1), int(math.floor(checkY)+1)))>0):
                    lineOk=False
            
            #update next value
            loc_x=loc_x+.25

    #print "end of line of sight\n"
    return lineOk

#---- Theta* Algorithm ---------------------------------------
#print str(lineOfSight(getVertexID(start[0], start[1]), getVertexID(goal[0], goal[1])))


print "\nStarting Theta*"

#initialize Theta* -------------------------------------------------
for i in range (graph.getNumVertices()):
    info=graph.getVertex(i)
    info[2]=1
    info[3]=0
    graph.setVertex(i, info)

pathCostT=[-1000000000 for x in range(graph.getNumVertices())] # g(n)
fringeTheta=[] #open list!
pathFoundTheta=False

# first takes an array: first value is g+h, second is vertex id
parentTheta=[-1 for x in range(graph.getNumVertices())]

pathCostT[getVertexID(start[0], start[1])]=0 #start following algorithm
parentTheta[getVertexID(start[0], start[1])]=getVertexID(start[0], start[1])

#push start into the queue --> heap assign an array as a value to each item. The array has the f(item), and the item's vertex id
heapq.heappush(fringeTheta, [pathCostT[getVertexID(start[0], start[1])]+getDistanceToGoal(start[0], start[1]),pathCostT[getVertexID(start[0], start[1])], getVertexID(start[0], start[1])])

#mark it as "added to fringe" by changing thrid # to 1
VertexInfo=graph.getVertex(getVertexID(start[0], start[1]))
VertexInfo[3]=1
graph.setVertex(getVertexID(start[0], start[1]), VertexInfo)
#--------------------------------------------------------------------

#Update Vertex for Theta* -----------------------------------------------
def UpdateVertexTheta(s, sPrime, queue):
    val=graph.getVertex(sPrime)
    parentS=int(parentTheta[s])
    parentSVal=graph.getVertex(parentS)
    
    #print "UpdateVertex: " + str(s) + " " + str(sPrime) + " " + str(parentTheta[s])
    if( lineOfSight( parentTheta[s], sPrime ) ==True):
        # Path 2
        if((pathCostT[parentS]+getDistance(parentSVal[0], parentSVal[1], val[0], val[1])) < pathCostT[sPrime]):
            pathCostT[sPrime] = pathCostT[parentS]+getDistance(parentSVal[0], parentSVal[1], val[0], val[1])
            parentTheta[sPrime] = parentTheta[s]
            #print str(parentTheta[sPrime]) + " " + str(sPrime)

            #  insert into open list
            checkVertex=graph.getVertex(sPrime)
            if(checkVertex[3]==1):
                cpyQueue=queue
                queue=removeFromHeap(cpyQueue, sPrime)
            distance=getDistanceToGoal(val[0],val[1])
            heapq.heappush(queue, [pathCostT[sPrime]+distance,pathCostT[sPrime], sPrime])
        
            #mark it, and give it f, g, h values
            VertexInfo=graph.getVertex(sPrime)
            VertexInfo[3]=1
            VertexInfo[7]=pathCostT[sPrime]
            VertexInfo[8]=distance
            VertexInfo[9]=pathCostT[sPrime]+distance
            graph.setVertex(sPrime, VertexInfo)
    
    else:
        # Path 1
        sVal=graph.getVertex(s)
        if( ( pathCostT[s]+getDistance(sVal[0], sVal[1], val[0], val[1])) < pathCostT[sPrime] ):
            pathCostT[sPrime] = pathCostT[s]+getDistance(sVal[0], sVal[1], val[0], val[1])
            parentTheta[sPrime] =s
            #print str(parentTheta[sPrime]) + " " + str(sPrime)

            #  insert into open list
            checkVertex=graph.getVertex(sPrime)
            if(checkVertex[3]==1):
                cpyQueue=queue
                queue=removeFromHeap(cpyQueue, sPrime)
            distance=getDistanceToGoal(val[0],val[1])
            heapq.heappush(queue, [pathCostT[sPrime]+distance,pathCostT[sPrime], sPrime])
        
            #mark vertex, and assign f,g, h
            VertexInfo=graph.getVertex(sPrime)
            VertexInfo[3]=1
            VertexInfo[7]=pathCostT[sPrime] #g
            VertexInfo[8]=distance #h
            VertexInfo[9]=pathCostT[sPrime]+distance
            graph.setVertex(sPrime, VertexInfo)
    return queue
#-----------------------------------------------------------------------

#-- Theta main----------------------------------------------------------
startTimeT=time.time()
while len(fringeTheta)!=0:
    #pop first value of queue --> checkout the beginning of "making a graph" to find out what's in the array
    vrtx=heapq.heappop(fringeTheta)
    # print "vrtx: " + str(vrtx)
    
    #if it's goal, then break
    if(vrtx[2]==getVertexID(goal[0], goal[1])):
        print "Theta* path is found!"
        print "Time taken: " +str(time.time()-startTimeT) 
        vrtInfo=graph.getVertex(vrtx[2])
        print "Total cost: " +str(vrtInfo[7])
        pathFoundTheta=True
    
    if(pathFoundTheta==True):
        break
    
    #mark vertex as closed
    VertexInfo=graph.getVertex(vrtx[2])
    VertexInfo[2]=0
    graph.setVertex(vrtx[2], VertexInfo)
    
    #get the neighbors
    ngbrs=[]
    ngbrs=graph.neighbourOf(vrtx[2])
    #print "neighbors: " + str(ngbrs)
    #loop through neighbors
    for i in range (len(ngbrs)):
        checkNeighbor=graph.getVertex(ngbrs[i])
        if(checkNeighbor[2]==1):
            #if not in fringe
            if(checkNeighbor[3]==0):
                pathCostT[ngbrs[i]]=1000000000
            cpyFringe=fringeTheta
            fringeTheta=UpdateVertexTheta(vrtx[2], ngbrs[i], cpyFringe)


if(pathFoundTheta!=True):
    print "Theta* did not find path!"

#-------------------------------------------- -----------------

#############################################################################
#############################################################################
#############################################################################
if(pathFoundTheta!=True and pathFound!=True):
    print "No paths found!"
    sys.exit()

pygame.init()
# Set the height and width of the screen
size = [1225, 625]
screen = pygame.display.set_mode(size)

pygame.display.set_caption("Path Visualization")

#Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

blockedDone=False
# -------- Main Program Loop ----------- ######################################
while done==False:
   for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done=True # Flag that we are done so we exit this loop
        elif event.type == pygame.MOUSEBUTTONDOWN:
            cursor = event.pos#pygame.mouse.get_pos()
            row_=int(math.floor(cursor[1]/(margin+width)))
            col_=int(math.floor(cursor[0]/(margin+width)))
            print("x: " + str(col_ ))
            print("y: " + str(row_ ))
            print("Vertex: " + str(getVertexID(col_, row_)))
            vertexInfo=graph.getVertex(getVertexID(col_, row_))
            print "A* --- f: " + str(vertexInfo[6]) + " g: " + str(vertexInfo[4] )+ " h: " +str(vertexInfo[5])
            print "Theta* --- f: " + str(vertexInfo[9]) + " g: " + str(vertexInfo[7] )+ " h: " +str(vertexInfo[8])
        #check for pressed key to pick start
        elif event.type == pygame.KEYDOWN:
             if pygame.key.name(event.key)=="return":
                 print "\nStarting Algorithm. . . "
                 done=True
                  
   # Set the screen background
   screen.fill(black)

   # ALL CODE TO DRAW SHOULD GO BELOW THIS COMMENT #############################
   offset = margin + width

   # Draw the grid  -----------------
   for row in range(rowLength):
      startY = row * offset
       
      for column in range(colLength):

        #check for blocked cell
       foundCell=False
       for [x, y] in obstacles:
        if [column, row]==[x,y]:
            foundCell=True          
          
       #make cell gray or white
       if column==0 or row==0 or column==(colLength-1) or row==(rowLength-1) or foundCell==True:
         sqrColor=gray
         pygame.draw.rect(screen,sqrColor,[(margin+width)*column+margin,(margin+height)*row+margin,width,height])

       else:
         sqrColor = white
         pygame.draw.rect(screen,sqrColor,[(margin+width)*column+margin,(margin+height)*row+margin,width,height])            
            

       # for drawing forward diagonals in the squares
         startX = column * offset

         if (column!=0 and row!=0 and column!=(colLength-1) and row!=(rowLength-1)) and foundCell==False:
             lineColor = black
             pygame.draw.line(screen, lineColor, [startX, startY], [startX + offset, startY + offset], margin)

   
   # draw the backward diagonals
   for row in range(rowLength):

      startY = row * offset

      for column in range(colLength):
        #check for blocked cell          
         foundCell=False
         for [x, y] in obstacles:
            if [column, row]==[x,y]:
                foundCell=True 
             
         # for drawing backward diagonals in the squares
         startX = offset + margin
         startX += column * offset

         if (column!=0 and row!=0 and column!=(colLength-1) and row!=(rowLength-1)) and foundCell==False:
            lineColor = black
            pygame.draw.line(screen, lineColor, [startX, startY], [startX - offset, startY + offset], margin)
  
   """
   #show if the edges are correctly connected -----------
   edges=graph.getAllEdges()
   for i in range(numEdges):
       #print "edge: " + str((edges[i][0]%(colLength-1))+1)+","+str((edges[i][0]/(colLength-1))+1)
       #print "edge: " + str((edges[i][1]%(colLength-1))+1)+","+str((edges[i][1]/(colLength-1))+1)
       xLoc1=(((edges[i][0]%(colLength-1))+1)*(width+margin))
       yLoc1=(((edges[i][0]/(colLength-1))+1)*(width+margin))
       xLoc2=(((edges[i][1]%(colLength-1))+1)*(width+margin))
       yLoc2=(((edges[i][1]/(colLength-1))+1)*(width+margin))
       pygame.draw.line(screen, red, (xLoc1, yLoc1), (xLoc2, yLoc2), 2)
   """


   child=getVertexID(goal[0], goal[1])
   parents=parent[child]
   if (parents!=-1):
       while(not(parents==getVertexID(start[0], start[1]) and child==getVertexID(start[0], start[1]))):
           xLoc1=(((child%(colLength-1))+1)*(width+margin))
           yLoc1=(((child/(colLength-1))+1)*(width+margin))
           xLoc2=(((parents%(colLength-1))+1)*(width+margin))
           yLoc2=(((parents/(colLength-1))+1)*(width+margin))
           pygame.draw.line(screen, red, (xLoc1, yLoc1), (xLoc2, yLoc2), 2) 
           child=parents
           parents=parent[child]

   child=getVertexID(goal[0], goal[1])
   parents=parentTheta[child]
   if (parents!=-1):
       while(not(parents==getVertexID(start[0], start[1]) and child==getVertexID(start[0], start[1]))):
            xLoc1=(((child%(colLength-1))+1)*(width+margin))
            yLoc1=(((child/(colLength-1))+1)*(width+margin))
            xLoc2=(((parents%(colLength-1))+1)*(width+margin))
            yLoc2=(((parents/(colLength-1))+1)*(width+margin))
            pygame.draw.line(screen, blue, (xLoc1, yLoc1), (xLoc2, yLoc2), 2) 
            child=parents
            parents=parentTheta[child]
    
    
       insert=(int(math.ceil(start[0]*(margin+width))), int(math.ceil(start[1]*(margin+width))))
       pygame.draw.circle(screen,green,insert,int(.5*width),int(.5))

       insert=(int(math.ceil(goal[0]*(margin+width))), int(math.ceil(goal[1]*(margin+width))))
       pygame.draw.circle(screen,red,insert,int(.5*width),int(.5))
    
   else:
     print "PATH NOT FOUND!" 
     done=True   

   #------------------------------
   #---------ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT --------

   # Limit to 20 frames per second
   clock.tick(9000)


   # update the screen
   pygame.display.flip()

#####---------END FIRST DISPLAY LOOP ---------#################################

# Be IDLE friendly. If you forget this line, the program will 'hang' on exit.
pygame.quit()
