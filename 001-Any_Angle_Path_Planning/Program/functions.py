# Authors: Tina Nye & Qandeel Sajid
# Project: AI-Project 1

#! /usr/bin/env python
# Pygame visualization template

import pygame
import math
import min_heap
import sys
import time


import random
from apgl.graph.SparseGraph import *
from apgl.graph.VertexList import *
import numpy
import scipy.sparse

##############################################################################
#-----CONSTANTS-------------------------------------------------
# Define some colors

# This sets the width and height of each grid location
width=11
height=11

# This sets the margin between each cell
margin=1

# Set the row and column length
rowLength = 52
colLength = 102
##############################################################################
#########################################################################
#---- Common Functions ---------------------------------------
#this just uses the x/y value of a vertex to give the distance to goal (heuristic)
def getDistanceToGoal(x,y, goal):
    return math.sqrt(math.pow((x-goal[0]),2)+math.pow((y-goal[1]),2))

def getDistance(x,y, x1, y1):
    if(not((x-x1==0) and (y-y1==0))):
        return math.sqrt(math.pow((x-x1),2)+math.pow((y-y1),2))
    elif(x-x1==0):
        return (y-y1==0)
    elif(y-y1==0):
        return (x-x1==0)

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
    for i in range (origQueue.len()):
        item=origQueue.get(i)
        if(value==item[2]):
            origQueue.remove(i)
            break
    return origQueue


def lineOfSight_modified(graph, s, sPrime, obstacles, old_graph):
        #print "\nstarting line of sight: " +str(s) + " "+str(sPrime)
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
                    if not(old_graph.getEdge(getVertexID(int(math.floor(checkX)), int(math.floor(checkY))), getVertexID(int(math.floor(checkX+1)), int(math.floor(checkY+1))))>0):
                        #print "doesn't work"
                        return False
                
                elif (not((checkX%1.0)==0.0 and (checkY%1.0)==0.0)):
                    #print "the else if"
                    for i in range (len(obstacles)):
                        value=[int(math.floor(checkX)),int(math.floor(checkY))]
                        if obstacles[i]==[int(math.floor(checkX)),int(math.floor(checkY))]:
                            #print "2doesn't work: "+str(sPrime)
                            return False
                                    
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
                #print "vertex: "+str(getVertexID(int(math.floor(checkX)), int(math.floor(checkY))))
                
                if(((loc_x%1.0)==0.0 and (loc_x%1.0)==0.0) and int(loc_x)==int(end_x) and int(loc_y)==int(end_y)):
                    break
                
                if (not((checkX%1.0)==0.0 or (checkY%1.0)==0.0) and (checkX+1)<(colLength-1) and (checkY+1)<(rowLength-1)):
                    #print "x: "+str(x) + " y: " + str(y) + " not whole"
                    if not(old_graph.getEdge(getVertexID(int(math.floor(checkX)), int(math.floor(checkY))), getVertexID(int(math.floor(checkX)+1), int(math.floor(checkY)+1)))>0):
                        return False
            
                elif (not((checkX%1.0)==0.0 and (checkY%1.0)==0.0)):
                    for i in range (len(obstacles)):
                        if obstacles[i]==[int(math.floor(checkX)),int(math.floor(checkY))]:
                            return False                
            
                #update next value
                loc_y=loc_y+.25
        
        elif((sPrime_y-sPrime_y)==0):
            #print "horizontal line"
            while(not(round(loc_x,10)==round(end_x,10) and round(loc_y, 10)==round  (end_y, 10))):
                x=convertPixelToCol(loc_x)
                y=convertPixelToRow(loc_y)
                checkX=x
                checkY=y
                #print "vertex: "+str(getVertexID(int(math.floor(checkX)), int(math.floor(checkY))))
                
                if(((loc_x%1.0)==0.0 and (loc_x%1.0)==0.0) and int(loc_x)==int(end_x) and int(loc_y)==int(end_y)):
                    break
                
                #check for blocked
                if (not((checkX%1.0)==0.0 or (checkY%1.0)==0.0) and (checkX+1)<(colLength-1) and (checkY+1)<(rowLength-1)):
                    #print "x: "+str(x) + " y: " + str(y) + " not whole"
                    #print str(x%1.0)+ " " + str(y%1.0)
                    #print "x: "+str(math.floor(checkX)) + " y: " + str(math.floor(checkY)) + " not whole"
                    if not(old_graph.getEdge(getVertexID(int(math.floor       (checkX)), int(math.floor(checkY))), getVertexID  (int(math.floor(checkX)+1), int(math.floor(checkY)+1)))>0):
                        #print "1doesn't work: "+str(sPrime)
                        return False
            
                elif (not((checkX%1.0)==0.0 and (checkY%1.0)==0.0)):
                    for i in range (len(obstacles)):
                        if obstacles[i]==[int(math.floor(checkX)),int(math.floor(checkY))]:
                            #print "2doesn't work: "+str(sPrime)
                            return False
                            
                #update next value
                loc_x=loc_x+.25
        
        #print "end of line of sight\n"
        return lineOk

