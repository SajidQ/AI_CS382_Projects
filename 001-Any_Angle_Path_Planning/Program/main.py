# Authors: Tina Nye & Qandeel Sajid
# Project: AI-Project 1

#! /usr/bin/env python
# Pygame visualization template

import pygame
import math
import min_heap
import sys
import time
import functions
import A_star
import Theta_star


import random
from apgl.graph.SparseGraph import *
from apgl.graph.VertexList import *
import numpy
import scipy.sparse

###### Print Intro
print "\n--------------------------------"
print "Program: Any-Angle Path Planning"
print "By: Tina Nye & Qandeel Sajid"

##############################################################################
#-----CONSTANTS-------------------------------------------------
# Define some colors
black = ( 0, 0, 0)
white = ( 255, 255, 255)
green = ( 0, 255, 0)
blue=(0,0,255)
red = ( 255, 0, 0)
gray=(88, 88, 88)
pink=(255,0,255)
light_blue=(0,255,255)
light_gray=(152,152,152)
percentBlocked=.1  #--10% 

# This sets the width and height of each grid location
width=11
height=11
#width=21
#height=21


# This sets the margin between each cell
margin=1

# Set the row and column length
rowLength = 52
colLength = 102
#rowLength = 26
#colLength = 51
file_provided=False

##############################################################################
#check if file is provided
#if so, get obstacles, start/goal from file

if(len(sys.argv)>1):
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

    
#-----------------------------------------------------------------
##############################################################################
##############################################################################
##############################################################################
#--- Randomly pick out 10% BLOCKED CELLS if file not provided ---
#This sections first randomly picks random vertices and makes them into obstacles. The if statement ensures that the border does not count as the 10% of blocked vertices
if(file_provided==False):
    obstacles=[]
    maxBlockCells=int(math.floor((rowLength-2)*(colLength-2)*percentBlocked))
    #print "number of blocked cells: " + str(maxBlockCells)

    while(len(obstacles)!=maxBlockCells):
        x=int(math.floor(random.random()*(colLength-1)))
        y=int(math.floor(random.random()*(rowLength-1)))
        #print str(x) +", y="+ str(y)
        if(obstacles.count([x,y])==0 and x!=0 and x!=colLength and y!=0 and y!= rowLength):
            obstacles.append([x,y])

    # this part blocks out the bordering cells
    for i in range(colLength):
        obstacles.append([i,0])
        obstacles.append([i, rowLength-1])

    for i in range(rowLength):
        obstacles.append([0, i])
        obstacles.append([colLength-1,i])

##############################################################################
##############################################################################
##############################################################################
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

##############################################################################
##############################################################################
# WRITE OUT TO FILE (write the obstacle out if the file is not provided)
"""
name="test_samples/"+str(t)+".txt"
print "Printing file: " + name
if(file_provided==False):
    fout=open(name, "w")
    for i in range (len(obstacles)):
        item=obstacles[i]
        fout.write(str(item[0]))
        fout.write(" ")
        fout.write(str(item[1]))
        fout.write("\n")

    #write out start
    fout.write(str(start[0]))
    fout.write(" ")
    fout.write(str(start[1]))
    fout.write("\n")

    #write out goal
    fout.write(str(goal[0]))
    fout.write(" ")
    fout.write(str(goal[1]))
    fout.write("\n")
    fout.close()
"""
##############################################################################
##############################################################################
##############################################################################
#parent list
parent=[-1 for x in range(graph.getNumVertices())]
parentTheta=[-1 for x in range(graph.getNumVertices())]
parentV=[-1 for x in range(graph.getNumVertices())]

#run A*
a_star=A_star.A_star(graph,[])
parent=a_star.run(start, goal)


#run Theta*
theta_star=Theta_star.Theta_star(graph,obstacles)
parentTheta=theta_star.run(start, goal)



#check if anyone found path
child=functions.getVertexID(goal[0], goal[1])
parents=parent[child]
parentT=parentTheta[child]
if (parents==-1 and parentT==-1):
    print "No paths found!"
    sys.exit()


#############################################################################
#############################################################################
#############################################################################
#run A* on visibility graph

corners=[start,goal]
for i in range (len(obstacles)):
    value=obstacles[i]
    if(value[0]!=0 and value[1]!=0 and value[0]!=(colLength-1) and value[1]!=(rowLength-1)):
        corners.append(value)
        corners.append([value[0]+1,value[1]])
        corners.append([value[0],value[1]+1])
        corners.append([value[0]+1,value[1]+1])

open_vrts=[]
count=0
for i in range (graph.getNumVertices()):
    value=graph.getVertex(i)
    found=False
    #check each corner
    for j in range (len(corners)):
        obs_val=corners[j]
        if(value[0]==obs_val[0] and value[1]==obs_val[1]):
            found=True
            break
    if(found!=False):
        open_vrts.append(i)
        count=count+1


#** make new graph
numFeatures=0
numVertices=(rowLength-1)*(colLength-1)
vList=GeneralVertexList(numVertices)
weightMatrix = scipy.sparse.lil_matrix((numVertices, numVertices))
graph_new = SparseGraph(vList,True, W=weightMatrix)  # HERE IS YOUR GRAPH!


#-- This assigns to each vertex an array as a value. In the array, the first value is the x location, and then y (row), and 1 implies the vertex is unexplored. 0 will mean it is closed. The fourth value determines whether the vertex is in the fringe (1) or not (0)
#-- 5th value-g(A*), h(A*), f(A*), g(Theta*), h(Theta*), f(Theta*)
row=1
for i in range (graph_new.getNumVertices()):
    if(i%(colLength-1)==0 and i!=0):
        row=row+1
    graph_new.setVertex(i, [(i%(colLength-1))+1,row, 1, 0,100000000000.0,0.0, 100000000000.0,100000000000.0, 0.0, 100000000000.0])



#print "old: " + str(graph.getNumEdges()) + " new graph: " + str(graph_new.getNumEdges())

#check is another file is available
edge_file=False
if(len(sys.argv)>2):
    try:
        fin=open(sys.argv[2], "r")
    except IOError:
        print '\nERROR: Cannot open file, will use random obstacles and start/goal'
    else:
        print sys.argv[2]
        edge_file=True
        
        #read every thing from file
        fileInfo=[]
        fileInfo.append(fin.readline())
        while fileInfo[len(fileInfo)-1]:
            fileInfo.append(fin.readline())
        
        # Get obstacles from file
        for i in range (len(fileInfo)-1):
            val= (fileInfo[i].split(' '))
            node1=int(val[0])
            node2=int(val[1])
            v1=graph_new.getVertex(node1)
            v2=graph_new.getVertex(node2)
            graph_new.addEdge(node1,node2, edge=functions.getDistance(v1[0],v1[1],v2[0],v2[1]))


#run A* on visibility graph
visibility_a_star=A_star.A_star(graph_new, open_vrts)
parentV=visibility_a_star.run_visibility(start, goal,obstacles, open_vrts,graph, edge_file)



"""
#---- out put the edges
name="test_samples/"+str(t)+"edges.txt"
print "Printing file: " + name
if(file_provided==False and edge_file==False):
    fout2=open(name, "w")
    edges=graph_new.getAllEdges();
    for i in range (len(edges)):
        item=edges[i]
        fout2.write(str(item[0]))
        fout2.write(" ")
        fout2.write(str(item[1]))
        fout2.write("\n")
    fout2.close()
"""



#############################################################################
#############################################################################
#############################################################################
#                                          UNCOMMENT THIS WHEN NOT SCRIPTING
pygame.init()
# Set the height and width of the screen
size = [colLength*(width+margin)+1, rowLength*(width+margin)+1]
#size=[613,313]
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
            print("Vertex: " + str(functions.getVertexID(col_, row_)))
            vertexInfo=graph.getVertex(functions.getVertexID(col_, row_))
            visibility_info=graph_new.getVertex(functions.getVertexID(col_, row_))
            print "A* --- f: " + str(vertexInfo[6]) + " g: " + str(vertexInfo[4] )+ " h: " +str(vertexInfo[5])
            print "Theta* --- f: " + str(vertexInfo[9]) + " g: " + str(vertexInfo[7] )+ " h: " +str(vertexInfo[8])
            print "Visibility+A* --- f: " + str(visibility_info[6]) + " g: " + str(visibility_info[4] )+ " h: " +str(visibility_info[5])
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


   edges=graph_new.getAllEdges();
   for i in range (len(edges)):
        node1=[(((edges[i][0]%(colLength-1))+1)*(width+margin)),(((edges[i][0]/(colLength-1))+1)*(width+margin))]
        node2=[(((edges[i][1]%(colLength-1))+1)*(width+margin)),(((edges[i][1]/(colLength-1))+1)*(width+margin))]
        #print str(node1)+" "+str(node2)
        pygame.draw.line(screen, black, (node1[0], node1[1]), (node2[0], node2[1]), 1)
        

       
   child=functions.getVertexID(goal[0], goal[1])
   parents=parent[child]
   if (parents!=-1):
       while(not(parents==functions.getVertexID(start[0], start[1]) and child==functions.getVertexID(start[0], start[1]))):
           xLoc1=(((child%(colLength-1))+1)*(width+margin))
           yLoc1=(((child/(colLength-1))+1)*(width+margin))
           xLoc2=(((parents%(colLength-1))+1)*(width+margin))
           yLoc2=(((parents/(colLength-1))+1)*(width+margin))
           pygame.draw.line(screen, red, (xLoc1, yLoc1), (xLoc2, yLoc2), 2) 
           child=parents
           parents=parent[child]
 
   
   #print the Theta*
   child=functions.getVertexID(goal[0], goal[1])
   parents=parentTheta[child]
   if (parents!=-1):
       while(not(parents==functions.getVertexID(start[0], start[1]) and child==functions.getVertexID(start[0], start[1]))):
            xLoc1=(((child%(colLength-1))+1)*(width+margin))
            yLoc1=(((child/(colLength-1))+1)*(width+margin))
            xLoc2=(((parents%(colLength-1))+1)*(width+margin))
            yLoc2=(((parents/(colLength-1))+1)*(width+margin))
            pygame.draw.line(screen, blue, (xLoc1, yLoc1), (xLoc2, yLoc2), 2) 
            child=parents
            parents=parentTheta[child]
   
                
   #print the Theta*
   child=functions.getVertexID(goal[0], goal[1])
   parents=parentV[child]
   if (parents!=-1):
       while(not(parents==functions.getVertexID(start[0], start[1]) and child==functions.getVertexID(start[0], start[1]))):
            xLoc1=(((child%(colLength-1))+1)*(width+margin))
            yLoc1=(((child/(colLength-1))+1)*(width+margin))
            xLoc2=(((parents%(colLength-1))+1)*(width+margin))
            yLoc2=(((parents/(colLength-1))+1)*(width+margin))
            pygame.draw.line(screen, light_blue, (xLoc1, yLoc1), (xLoc2, yLoc2), 2) 
            child=parents
            parents=parentV[child]

   insert=(int(math.ceil(start[0]*(margin+width))), int(math.ceil(start[1]*(margin+width))))
   pygame.draw.circle(screen,green,insert,int(.5*width),int(.5))

   insert=(int(math.ceil(goal[0]*(margin+width))), int(math.ceil(goal[1]*(margin+width))))
   pygame.draw.circle(screen,pink,insert,int(.5*width),int(.5))
    

   #------------------------------
   #---------ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT --------

   # Limit to 20 frames per second
   clock.tick(9000)


   # update the screen
   pygame.display.flip()

#####---------END FIRST DISPLAY LOOP ---------#################################

# Be IDLE friendly. If you forget this line, the program will 'hang' on exit.
pygame.quit()

       

       
