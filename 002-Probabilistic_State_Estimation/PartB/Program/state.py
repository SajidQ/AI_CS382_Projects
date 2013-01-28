

class State():
   '''
      Class:
      cotains the state of the object
      as position and direction (angle object is facing)
      '''

   def __init__(self, x, y, theta):
      self.x = x
      self.y = y
      self.theta = theta
   

   def __str__(self):  # print the state
      return "(%s, %s, %s)"%(self.x, self.y, self.theta)

   def set_angle(self, angle):
      self.theta=angle

   def print_state(self, fout):
      fout.write("\n"+str(self.x) + " " + str(self.y) + " " +str(self.theta))
      


class Path():
   def __init__(self):
      self.stateList = []

   def append_state(self, state):
      if(state!=None and len(self.stateList)!=0):
         self.stateList.append(state)
      if(len(self.stateList)==0):
         state_=State(10, 10, 0)
         self.stateList.append(state_)
      

   def get_Path(self):
      return self.stateList

   def output_list(self):
      fout=open("output/path.txt", "w")
      fout.write(str(len(self.stateList)))
      for i in range (len(self.stateList)):
        self.stateList[i].print_state(fout)

      print "path.txt outputted\n"

   def get_prev_state(self):
      return self.stateList[len(self.stateList)-1]

class Landmark():
   def __init__(self, id, position):
      self.id = id
      self.position = position

   def get_id(self):
      return self.id

   def get_position(self):
      return self.position
         
