import functions
import min_heap
import time


# Set the row and column length
rowLength = 52
colLength = 102

class A_star:
    def __init__(self, graph,open_vrts):
        self.pathCost=[-1000000000 for x in range(graph.getNumVertices())] # g(n)
        self.fringe=min_heap.min_heap() #open list!
        self.pathFound=False
        # first takes an array: first value is g+h, second is vertex id
        self.parent=[-1 for x in range(graph.getNumVertices())]
        self.graph=graph
        if (open_vrts!=[]):
            for i in range (len(open_vrts)):
                info=self.graph.getVertex(open_vrts[i])
                info[2]=1
                info[3]=0
                self.graph.setVertex(open_vrts[i], info)
    
    #--------------------------------------------
    def UpdateVertex(self,s, sPrime, queue,goal):
        val=self.graph.getVertex(sPrime)
        if((self.pathCost[s]+self.graph.getEdge(s, sPrime))<self.pathCost[sPrime]):
            self.pathCost[sPrime]=self.pathCost[s]+self.graph.getEdge(s, sPrime)
            self.parent[sPrime]=s
            
            #remove from queue
            cpyQueue=queue
            queue=functions.removeFromHeap(cpyQueue, sPrime)
            distance=functions.getDistanceToGoal(val[0],val[1], goal)
            queue.push([self.pathCost[sPrime]+distance,self.pathCost[sPrime], sPrime, True])
            VertexInfo=self.graph.getVertex(sPrime)
            VertexInfo[3]=1
            VertexInfo[4]=self.pathCost[sPrime]
            VertexInfo[5]=distance
            VertexInfo[6]=self.pathCost[sPrime]+distance
            self.graph.setVertex(sPrime, VertexInfo)
        
        #print "cost of: " + str(sPrime) + " " + str(self.pathCost[sPrime]+getDistanceToGoal(val[0],val[1]))
        return queue

    def get_graph(self):
        return self.graph

    #-----------------------------------------------
    def run(self, start, goal):
        self.pathCost[functions.getVertexID(start[0], start[1])]=0 #start following algorithm
        self.parent[functions.getVertexID(start[0], start[1])]=functions.getVertexID(start[0], start[1])

        #push start into the queue --> heap assign an array as a value to each item. The array has the f(item), and the item's vertex id

        self.fringe.push([self.pathCost[functions.getVertexID(start[0], start[1])]+functions.getDistanceToGoal(start[0], start[1], goal),self.pathCost[functions.getVertexID(start[0], start[1])], functions.getVertexID(start[0], start[1]), True])
        
        #mark it as "added to self.fringe" by changing thrid # to 1
        VertexInfo=self.graph.getVertex(functions.getVertexID(start[0], start[1]))
        VertexInfo[3]=1
        self.graph.setVertex(functions.getVertexID(start[0], start[1]), VertexInfo)
            
        #start the A* loop
        print "\nStarting A*"
        startTimeA=time.time()
        while (self.fringe.len)>0:
            #pop first value of queue --> checkout the beginning of "making a self.graph" to find out what's in the array
            vrtx=self.fringe.pop()
            #print "vrtx: " + str(vrtx)
            if(vrtx==-1):
                self.pathFound=False
                break
    
            #if it's goal, then break
            if(vrtx[2]==functions.getVertexID(goal[0], goal[1])):
                print "A* found path!"
                print "Time taken: " +str(time.time()-startTimeA)
                vrtInfo=self.graph.getVertex(vrtx[2])
                print "Total cost: " +str(vrtInfo[4])
                self.pathFound=True
                #self.write_results(time.time()-startTimeA, vrtInfo[4], False)
                break
    
            #mark vertex as closed
            VertexInfo=self.graph.getVertex(vrtx[2])
            VertexInfo[2]=0
            self.graph.setVertex(vrtx[2], VertexInfo)
    
            #get the neighbors
            ngbrs=[]
            ngbrs=self.graph.neighbourOf(vrtx[2])
            #print "neighbors: " + str(ngbrs)
            #loop through neighbors
            for i in range (len(ngbrs)):
                checkNeighbor=self.graph.getVertex(ngbrs[i])
                if(checkNeighbor[2]==1):
                    if(checkNeighbor[3]==0):
                        self.pathCost[ngbrs[i]]=1000000000
                        self.parent[ngbrs[i]]=-1
                    cpyfringe=self.fringe
                    self.fringe=self.UpdateVertex(vrtx[2],ngbrs[i], cpyfringe, goal)

        if(self.pathFound!=True):
            print "A* did not find path!"
            return [-1 for x in range(self.graph.getNumVertices())]
        else:
            return self.parent


    #-----------------------------------------------
    def run_visibility(self, start, goal,obstacles, open_vrts, old_graph, file_provided):
        self.pathCost[functions.getVertexID(start[0], start[1])]=0 #start following algorithm
        self.parent[functions.getVertexID(start[0], start[1])]=functions.getVertexID(start[0], start[1])
        
        #push start into the queue --> heap assign an array as a value to each item. The array has the f(item), and the item's vertex id
        
        self.fringe.push([self.pathCost[functions.getVertexID(start[0], start[1])]+functions.getDistanceToGoal(start[0], start[1], goal),self.pathCost[functions.getVertexID(start[0], start[1])], functions.getVertexID(start[0], start[1]), True])
        
        #mark it as "added to self.fringe" by changing thrid # to 1
        VertexInfo=self.graph.getVertex(functions.getVertexID(start[0], start[1]))
        VertexInfo[3]=1
        self.graph.setVertex(functions.getVertexID(start[0], start[1]), VertexInfo)
        
        if(file_provided==False):
            for j in range (len(open_vrts)):
                if(open_vrts[j]!=-1):
                    if(not(self.graph.getEdge(functions.getVertexID(start[0], start[1]), open_vrts[j])) and functions.lineOfSight_modified(self.graph,functions.getVertexID(start[0], start[1]), open_vrts[j], obstacles, old_graph)):
                        v1=self.graph.getVertex(functions.getVertexID(start[0], start[1]))
                        v2=self.graph.getVertex(open_vrts[j])
                        self.graph.addEdge(functions.getVertexID(start[0], start[1]),open_vrts[j], edge=functions.getDistance(v1[0],v1[1],v2[0],v2[1]))
        
        #start the A* loop
        print "\nStarting Visibility A*"
        startTimeA=time.time()
        while (self.fringe.len)>0:
            #pop first value of queue --> checkout the beginning of "making a self.graph" to find out what's in the array
            vrtx=self.fringe.pop()
            #print "vrtx: " + str(vrtx)
            if(vrtx==-1):
                self.pathFound=False
                break
            
            #if it's goal, then break
            if(vrtx[2]==functions.getVertexID(goal[0], goal[1])):
                print "Visibility Graph-A* found path!"
                print "Time taken: " +str(time.time()-startTimeA)
                vrtInfo=self.graph.getVertex(vrtx[2])
                print "Total cost: " +str(vrtInfo[4])
                self.pathFound=True
                #self.write_results(time.time()-startTimeA, vrtInfo[4], True)
                break
            
            #mark vertex as closed
            VertexInfo=self.graph.getVertex(vrtx[2])
            VertexInfo[2]=0
            self.graph.setVertex(vrtx[2], VertexInfo)

            if(file_provided==False):
                for j in range (len(open_vrts)):
                    if(open_vrts [j]!=-1):
                        if(not(self.graph.getEdge(vrtx[2], open_vrts [j])) and functions.lineOfSight_modified(self.graph,vrtx[2], open_vrts [j], obstacles, old_graph)):
                            v1=self.graph.getVertex(vrtx[2])
                            v2=self.graph.getVertex(open_vrts [j])
                            self.graph.addEdge(vrtx[2],open_vrts [j], edge=functions.getDistance(v1[0],v1[1],v2[0],v2[1]))
            
            #get the neighbors
            ngbrs=[]
            ngbrs=self.graph.neighbourOf(vrtx[2])
            #print "neighbors: " + str(ngbrs)
            #loop through neighbors
            for i in range (len(ngbrs)):
                checkNeighbor=self.graph.getVertex(ngbrs[i])
                if(checkNeighbor[2]==1):
                    if(checkNeighbor[3]==0):
                        self.pathCost[ngbrs[i]]=1000000000
                        self.parent[ngbrs[i]]=-1
                    cpyfringe=self.fringe
                    self.fringe=self.UpdateVertex(vrtx[2],ngbrs[i], cpyfringe, goal)
        
        if(self.pathFound!=True):
            print "Visibility Graph with A* did not find path!"
            return [-1 for x in range(self.graph.getNumVertices())]
        else:
            return self.parent
    

    def write_results(self,time, cost, visibility):
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
            fout2.write("A*")
            fout2.write("Time taken: ")
            fout2.write(time)
            fout2.write("\n")
            fout2.write("Total cost: ")
            fout2.write(cost)
            fout2.write("\n\n")
            if(visibility==True):
                fout2.write("\n\n\n")
            fout2.close()

