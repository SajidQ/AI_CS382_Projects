import functions
import min_heap
import time
import sys
import math

# Set the row and column length
rowLength = 52
colLength = 102


class Theta_star:
    def __init__(self, graph, obstacles):
        self.pathCostT=[-1000000000 for x in range(graph.getNumVertices())] # g(n)
        self.fringeTheta=min_heap.min_heap() #open list!
        self.pathFoundTheta=False
        # first takes an array: first value is g+h, second is vertex id
        self.parentTheta=[-1 for x in range(graph.getNumVertices())]
        self.graph=graph
        self.obstacles=obstacles
        for i in range (self.graph.getNumVertices()):
            info=self.graph.getVertex(i)
            info[2]=1
            info[3]=0
            self.graph.setVertex(i, info)
    
  
    # --Line of Sight --------------------------------------
    def lineOfSight(self, s, sPrime, obstacles):
        #print "\nstarting line of sight"
        lineOk=True
        temp=self.graph.getVertex(s)
        s_x=functions.convertColToPixel(temp[0])
        s_y=functions.convertRowToPixel(temp[1])
        temp=self.graph.getVertex(sPrime)
        sPrime_x=functions.convertColToPixel(temp[0])
        sPrime_y=functions.convertRowToPixel(temp[1])
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
                x=functions.convertPixelToCol(loc_x)
                y=functions.convertPixelToRow(loc_y)
                checkX=x
                checkY=y
            
                #check for blocked
                if (not((checkX%1.0)==0.0 or (checkY%1.0)==0.0) and (checkX+1)<(colLength-1) and (checkY+1)<(rowLength-1)):
                    #print "x: "+str(x) + " y: " + str(y) + " not whole"
                    #print str(checkX%1.0)+ " " + str(checkY%1.0)
                    #print "x: "+str(math.floor(checkX)) + " y: " + str(math.floor(checkY)) + " not whole"
                    if not(self.graph.getEdge(functions.getVertexID(int(math.floor(checkX)), int(math.floor(checkY))), functions.getVertexID(int(math.floor(checkX+1)), int(math.floor(checkY+1))))>0):
                        #print "doesn't work"
                        return False
            
                elif (not((checkX%1.0)==0.0 and (checkY%1.0)==0.0)):
                    for i in range (len(obstacles)):
                        value=[int(math.floor(checkX)),int(math.floor(checkY))]
                        if obstacles[i]==[int(math.floor(checkX)),int(math.floor(checkY))]:
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
                x=functions.convertPixelToCol(loc_x)
                y=functions.convertPixelToRow(loc_y)
                checkX=x
                checkY=y    
            
                if(((loc_x%1.0)==0.0 and (loc_x%1.0)==0.0) and int(loc_x)==int(end_x) and int(loc_y)==int(end_y)):
                    break
            
                if (not((checkX%1.0)==0.0 or (checkY%1.0)==0.0) and (checkX+1)<(colLength-1) and (checkY+1)<(rowLength-1)):
                    #print "x: "+str(x) + " y: " + str(y) + " not whole"
                    #print str(x%1.0)+ " " + str(y%1.0)
                    #print "x: "+str(math.floor(checkX)) + " y: " + str(math.floor(checkY)) + " not whole"
                    if not(self.graph.getEdge(functions.getVertexID(int(math.floor(checkX)), int(math.floor(checkY))), functions.getVertexID(int(math.floor(checkX)+1), int(math.floor(checkY)+1)))>0):
                        #print "doesn't work"
                        return False
            
                elif (not((checkX%1.0)==0.0 and (checkY%1.0)==0.0)):
                    for i in range (len(obstacles)):
                        value=[int(math.floor(checkX)),int(math.floor(checkY))]
                        if obstacles[i]==[int(math.floor(checkX)),int(math.floor(checkY))]:
                            return False  
            
                #update next value
                loc_y=loc_y+.25
        
        elif((sPrime_y-sPrime_y)==0):
            #print "horizontal line"
            while(not(round(loc_x,10)==round(end_x,10) and round(loc_y, 10)==round  (end_y, 10))):
                #print "Pix_x: "+str(loc_x) + " Pix_y: " + str(loc_y)
                x=functions.convertPixelToCol(loc_x)
                y=functions.convertPixelToRow(loc_y)
                checkX=x
                checkY=y
            
                if(((loc_x%1.0)==0.0 and (loc_x%1.0)==0.0) and int(loc_x)==int(end_x) and int(loc_y)==int(end_y)):
                    break
            
                #check for blocked
                if (not((checkX%1.0)==0.0 or (checkY%1.0)==0.0) and (checkX+1)<(colLength-1) and (checkY+1)<(rowLength-1)):
                    #print "x: "+str(x) + " y: " + str(y) + " not whole"
                    #print str(x%1.0)+ " " + str(y%1.0)
                    #print "x: "+str(math.floor(checkX)) + " y: " + str(math.floor(checkY)) + " not whole"
                    if not(self.graph.getEdge(functions.getVertexID(int(math.floor       (checkX)), int(math.floor(checkY))), functions.getVertexID  (int(math.floor(checkX)+1), int(math.floor(checkY)+1)))>0):
                        #print "doesn't work"
                        return False
            
                elif (not((checkX%1.0)==0.0 and (checkY%1.0)==0.0)):
                    for i in range (len(obstacles)):
                        value=[int(math.floor(checkX)),int(math.floor(checkY))]
                        if obstacles[i]==[int(math.floor(checkX)),int(math.floor(checkY))]:
                            return False  
    
                #update next value
                loc_x=loc_x+.25
    
        #print "end of line of sight\n"
        return lineOk
    
    
    #--------------------------------------------
    def UpdateVertexTheta(self,s, sPrime, queue, goal):
        val=self.graph.getVertex(sPrime)
        parentS=int(self.parentTheta[s])
        parentSVal=self.graph.getVertex(parentS)
    
        #print "UpdateVertex: " + str(s) + " " + str(sPrime) + " " + str(self.parentTheta[s])
        if( self.lineOfSight( self.parentTheta[s], sPrime,self.obstacles) ==True):
            # Path 2
            if((self.pathCostT[parentS]+functions.getDistance(parentSVal[0], parentSVal[1], val[0], val[1])) < self.pathCostT[sPrime]):
                self.pathCostT[sPrime] = self.pathCostT[parentS]+functions.getDistance(parentSVal[0], parentSVal[1], val[0], val[1])
                self.parentTheta[sPrime] = self.parentTheta[s]
                #print str(self.parentTheta[sPrime]) + " " + str(sPrime)
            
                #  insert into open list
                checkVertex=self.graph.getVertex(sPrime)
                if(checkVertex[3]==1):
                    cpyQueue=queue
                    queue=functions.removeFromHeap(cpyQueue, sPrime)
                distance=functions.getDistanceToGoal(val[0],val[1], goal)
                queue.push([self.pathCostT[sPrime]+distance,self.pathCostT[sPrime], sPrime, True])
            
                #mark it, and give it f, g, h values
                VertexInfo=self.graph.getVertex(sPrime)
                VertexInfo[3]=1
                VertexInfo[7]=self.pathCostT[sPrime]
                VertexInfo[8]=distance
                VertexInfo[9]=self.pathCostT[sPrime]+distance
                self.graph.setVertex(sPrime, VertexInfo)
    
        else:
            # Path 1
            sVal=self.graph.getVertex(s)
            if( ( self.pathCostT[s]+functions.getDistance(sVal[0], sVal[1], val[0], val[1])) < self.pathCostT[sPrime] ):
                self.pathCostT[sPrime] = self.pathCostT[s]+functions.getDistance(sVal[0], sVal[1], val[0], val[1])
                self.parentTheta[sPrime] =s
                #print str(self.parentTheta[sPrime]) + " " + str(sPrime)
            
                #  insert into open list
                checkVertex=self.graph.getVertex(sPrime)
                if(checkVertex[3]==1):
                    cpyQueue=queue
                    queue=functions.removeFromHeap(cpyQueue, sPrime)
                distance=functions.getDistanceToGoal(val[0],val[1], goal)
                queue.push([self.pathCostT[sPrime]+distance,self.pathCostT[sPrime], sPrime, True])
            
                #mark vertex, and assign f,g, h
                VertexInfo=self.graph.getVertex(sPrime)
                VertexInfo[3]=1
                VertexInfo[7]=self.pathCostT[sPrime] #g
                VertexInfo[8]=distance #h
                VertexInfo[9]=self.pathCostT[sPrime]+distance
                self.graph.setVertex(sPrime, VertexInfo)
        return queue

    #-----------------------------------------------
    def run(self, start, goal):
        self.pathCostT[functions.getVertexID(start[0], start[1])]=0 #start following algorithm
        self.parentTheta[functions.getVertexID(start[0], start[1])]=functions.getVertexID(start[0], start[1])

        #push start into the queue --> heap assign an array as a value to each item. The array has the f(item), and the item's vertex id
        self.fringeTheta.push([self.pathCostT[functions.getVertexID(start[0], start[1])]+functions.getDistanceToGoal(start[0], start[1], goal),self.pathCostT[functions.getVertexID(start[0], start[1])], functions.getVertexID(start[0], start[1]), True])

        #mark it as "added to fringe" by changing thrid # to 1
        VertexInfo=self.graph.getVertex(functions.getVertexID(start[0], start[1]))
        VertexInfo[3]=1
        self.graph.setVertex(functions.getVertexID(start[0], start[1]), VertexInfo)

            
        #start the A* loop
        print "\nStarting Theta*"
        startTimeT=time.time()
        while self.fringeTheta.len()!=0:
            #pop first value of queue --> checkout the beginning of "making a self.graph" to find out what's in the array
            vrtx=self.fringeTheta.pop()
            #print "vrtx: " + str(vrtx)
    
            #if it's goal, then break
            if(vrtx[2]==functions.getVertexID(goal[0], goal[1])):
                print "Theta* path is found!"
                print "Time taken: " +str(time.time()-startTimeT) 
                vrtInfo=self.graph.getVertex(vrtx[2])
                print "Total cost: " +str(vrtInfo[7])
                self.pathFoundTheta=True
                #self.write_results(time.time()-startTimeT, vrtInfo[7])
                break
    
            #mark vertex as closed
            VertexInfo=self.graph.getVertex(vrtx[2])
            VertexInfo[2]=0
            self.graph.setVertex(vrtx[2], VertexInfo)
    
            #get the neighbors
            ngbrs=[]
            ngbrs=self.graph.neighbourOf(vrtx[2])
            #print "neighbors: " + str(ngbrs) + " size: " + str(len(ngbrs))
            #loop through neighbors
            for i in range (len(ngbrs)):
                checkNeighbor=self.graph.getVertex(ngbrs[i])
                if(checkNeighbor[2]==1):
                    #if not in fringe
                    if(checkNeighbor[3]==0):
                        self.pathCostT[ngbrs[i]]=10000000000
                        self.parentTheta[ngbrs[i]]=-1
                    cpyFringe=self.fringeTheta
                    self.fringeTheta=self.UpdateVertexTheta(vrtx[2], ngbrs[i], cpyFringe, goal)
               
        if(self.pathFoundTheta!=True):
            print "Theta* did not find path!"
            return [-1 for x in range(self.graph.getNumVertices())]

        else:
            return self.parentTheta

    def write_results(self,time, cost):
        #read everything
        try:
            fin=open("test_samples/results.txt", "r")
        except IOError:
            print '\nERROR: Cannot open file, will use random obstacles and start/goal'
        else:
            #read every thing from file
            fileInfo=[]
            fileInfo.append(fin.readline())
            while fileInfo[len(fileInfo)-1]:
                fileInfo.append(fin.readline())
            fin.close()
            
            #write every thing back
            fout2=open("test_samples/results.txt", "w")
            for i in range (len(fileInfo)):
                fout2.write(fileInfo[i])
    
    
            #add new
            fout2.write("Theta*")
            fout2.write("Time taken: ")
            fout2.write(time)
            fout2.write("\n")
            fout2.write("Total cost: ")
            fout2.write(cost)
            fout2.write("\n\n")
            fout2.close()

