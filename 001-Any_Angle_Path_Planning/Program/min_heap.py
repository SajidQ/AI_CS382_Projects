import math


class min_heap:
    def __init__(self):
        self.heap=[]
    
    #get values parent/children
    def parent(self,index):
        return int(math.floor(index/2))

    def left_child(self,index):
        return (2*index)+1

    def right_child(self,index):
        return (2*index)+2

    #compare value and move node up
    def up(self, index):
        while(index!=0 and self.heap[self.parent(index)] > self.heap[index]):
                save=self.heap[index]
                self.heap[index]=self.heap[self.parent(index)]
                self.heap[self.parent(index)]=save
                index=self.parent(index)
                
    def push(self, item):
        self.heap.append(item)
        self.up(len(self.heap)-1)

    #get value and remove it from heap
    def pop(self):
        if(len(self.heap)>0):
            top=self.heap[0]
            self.heap.remove(top)
            return top
        else:
            return -1

    def len(self):
        return len(self.heap)

    def get(self, index):
        return self.heap[index]

    def remove(self, index):
        value=self.heap[index]
        self.heap.remove(value)

    def printHeap(self):
        print str(self.heap)
        