import state
import observation
import gaussian
import math
import pygame
import random
import copy

BLACK = ( 0, 0, 0)
WHITE = ( 255, 255, 255)
GREEN = ( 0, 255, 0)
RED = ( 255, 0, 0)
BLUE=(0,0,255)
GRAY=(88, 88, 88)
PINK=(255,0,255)
LBLUE=(0,255,255)

class Particle():
   def __init__(self):
      self.path = state.Path()
      self.weight = 1
      self.index = 0
      self.avg_difference=0

   def append_state(self, state_):
      self.path.append_state(state_)

   def copy_particle(self, sent, index):
      self.weight=copy.deepcopy(sent.weight)
      self.index=index
      self.avg_difference=copy.deepcopy(sent.avg_difference)
      
      sent_path=sent.path.stateList
      self.path.stateList=[]
      for i in sent_path:
         state_=state.State(copy.deepcopy(i.x),copy.deepcopy(i.y),copy.deepcopy(i.theta))
         self.path.stateList.append(state_)
         

class Particle_Filter():
   def __init__(self):
      self.particles=[]
      self.observations=[]
   

   def angular_summation(self,angle, difference):
      ang=math.degrees(angle)
      while(difference < 0): 
         difference += 360;
      #print "+", difference
      while (difference > 0): 
         difference -= 360;
         sum = -difference+ang;
         if(sum>360):
            sum-=360
         return sum

   def get_distance(self,position, target):
      return math.sqrt(((position[0]-target[0])*(position[0]-target[0]))+((position[1]-target[1])*(position[1]-target[1])))
            
   def get_angle(self,current, next):
      angle=0
      if((current[0]-next[0]) >0):
         if((current[1]-next[1]) >0):
            angle=math.atan2((current[1]-next[1]),(current[0]-next[0]))
            angle=180-math.degrees(angle)
         #print "angle(-y): "+ str(angle)
         elif((current[1]-next[1]) <0):
            angle=math.atan2((current[1]-next[1]),(current[0]-next[0]))
            angle=180-math.degrees(angle)
         #print "angle(+y): "+ str(angle)
         else:
            angle=180
      elif((current[0]-next[0]) <0):
         if((current[1]-next[1]) >0):
            angle=math.atan2((current[1]-next[1]),(current[0]-next[0]))
            angle=180-math.degrees(angle)
         #print "angle(>y-): "+ str(angle)
         elif((current[1]-next[1]) <0):
            angle=math.atan2((current[1]-next[1]),(current[0]-next[0]))
            angle=180-math.degrees(angle)
         #print "angle(>y+): "+ str(angle)
         else:
            angle=360
      else:
         #print "angle(x=0): "+ str(angle)
         if((current[1]-next[1]) >0):
            angle=90
         elif((current[1]-next[1]) <0):
            angle=270
      
      #print str(angle)
      return angle

   def transition_model(self, prev_state, observed):
      #get values from previous state
      prev_angle=math.radians(prev_state.theta)
      prev_x=prev_state.x
      prev_y=prev_state.y

      #compute new values
      cos_angle=math.cos(prev_angle)
      sin_angle=-math.sin(prev_angle)
      diff=observed.disp+gaussian.main(0,.25)
      
      x_i=prev_x+(diff*cos_angle)
      y_i=prev_y+(diff*sin_angle)
      
      observed_angle=(math.radians(observed.rot)+gaussian.main(0,.01))
      angle_i=self.angular_summation(prev_angle, math.degrees(observed_angle))
                                     
      #print x_i,y_i,angle_i
      return state.State(x_i,y_i,angle_i)

   def observation_model(self, state_, landmarks_list,evidence):
      observed_landmarks=[]
      current_pos=[state_.x, state_.y]
      current_rot=math.radians(state_.theta)
      probability_1=0
      right_landmarks_1=0
      right_landmarks_2=0
      right_landmarks_3=0
      total_landmarks=len(evidence)
      
      if(total_landmarks==0):
         return 0.5
      
      
      #check if the evidence landmarks where observed within 90 degrees
      for i in evidence:
         max_1=math.degrees(i[1])+45
         min_1=math.degrees(i[1])-45
         max_2=math.degrees(i[1])+20
         min_2=math.degrees(i[1])-20
         max_3=math.degrees(i[1])+10
         min_3=math.degrees(i[1])-10
         
         for j in landmarks_list:
            if(i[0]==j.get_id()):
               land_pos=j.get_position()
               land_dist=self.get_distance(current_pos, land_pos)
               land_angle=math.radians(self.get_angle(current_pos, land_pos))
               land_angle=math.degrees(land_angle-current_rot)
               
               #check if the angle is a correct difference
               if(land_angle<=max_1 and land_angle>=min_1):
                  right_landmarks_1+=1
      
               if(land_angle<=max_2 and land_angle >=min_2):
                  right_landmarks_2+=1

               if(land_angle<=max_3 and land_angle >=min_3):
                  right_landmarks_3+=1

      return ((float(right_landmarks_1)/float(total_landmarks))*(float(right_landmarks_2)/float(total_landmarks))*(float(right_landmarks_3)/float(total_landmarks)))


   def read_sensing_input(self, name, file_num):
      #try to open the file
      sensing_name="output/sensing_"+str(file_num)+"_"+str(name)+".txt"
      fin=open(sensing_name, "r")

      #read path length
      path_length=int(fin.readline())
   
      #iterate through path
      for i in range(path_length):
         observe=copy.deepcopy(observation.Observation())
         pos=fin.readline()
         pos=pos.split(' ')
         
         #get displacement, theta, and #of landmarks
         observe.disp=float(pos[0])
         observe.rot=float(pos[1])
         observe.numLandmark=int(pos[2])
         
         #print observe.disp, observe.rot, observe.numLandmark,
         index=0
         for j in range(observe.numLandmark):
            #get id
            id=pos[3+index]
            id=id.split('[')
            id=id[1].split(',')
            id=int(id[0])
            #print id,

            index+=1
            #get orientation
            orient=pos[3+index]
            orient=orient.split(']')
            orient=float(orient[0])
            #print orient
            index+=1
            observe.append_landmark(id,orient)
         
         #save observation
         self.observations.append(observe)

   def get_avg_distance(self,ground_truth):
      #for every particle
      for i in self.particles:
         dist=0
         particle_path=copy.deepcopy(i.path.stateList)
         ground_path=copy.deepcopy(ground_truth.stateList)
         
         #find the average difference between the particle path & ground path
         for j in range (len(particle_path)):
            #print "path", len(particle_path), len(ground_path)
            points_p=[particle_path[j].x,particle_path[j].y]
            points_g=[ground_path[j].x,ground_path[j].y]
            dist+=self.get_distance(points_g,points_p)

         i.avg_difference=dist/float(len(particle_path))
            
            

   def algorithm(self, numParticles, filename, file_num, landmarks_list,linePoints,ground_truth):
      initial_state=state.State(10,10,0)
      
      #get observations
      self.read_sensing_input(filename,file_num)
      
      #insert particles
      for i in range(numParticles):
         particle=copy.deepcopy(Particle())
         particle.index=i
         self.particles.append(particle)
      
   
      #start iteration & visualization
      list_colors=[BLUE, GREEN, PINK, GRAY, LBLUE, RED]

      #Loop until the user clicks the close button ----------------------------------
      size = [500,500]
      screen = pygame.display.set_mode(size)
      pygame.display.set_caption("Path Visualization")
      done = False

      # Used to manage how fast the screen updates
      clock = pygame.time.Clock()

      state_num=0
      while done==False:
         for event in pygame.event.get(): # User did something
            if event.type == pygame.QUIT: # If user clicked close
               done=True # Flag that we are done so we exit this loop

         # Set the screen background
         screen.fill(WHITE)

         #do iteration
         for i in range (len(self.observations)):
            #get the evidence
            observed_landmarks=copy.deepcopy(self.observations[i])
            print "Observation: ",i
            
            sampling_list=[]
            for j in range (numParticles): #--------------------------------------------
               
               #print "particle: ", j, self.particles[j]
               #implement the transition model
               if(i==0):
                  new_state=copy.deepcopy(self.transition_model(initial_state, self.observations[i]))
                 
               else:
                  new_state=copy.deepcopy(self.transition_model(self.particles[j].path.get_prev_state(), self.observations[i]))
               
               self.particles[j].append_state(new_state)
            
               #get observations
               self.particles[j].weight=self.observation_model(new_state, landmarks_list,self.observations[i].landmarkPair)

               if(self.particles[j].weight>0):
                  part=copy.deepcopy(self.particles[j])
                  sampling_list.append(part)

               #display points
               sqrColor=list_colors[j%len(list_colors)]
               pygame.draw.rect(screen,sqrColor,[new_state.x,new_state.y,1,1], 2)
      
            for l in landmarks_list:
               pos=l.get_position()
               sqrColor=BLACK
               pygame.draw.rect(screen,sqrColor,[pos[0],pos[1],1,1], 2)
                        
            clock.tick(1)
            # Go ahead and update the screen with what we've drawn.
            pygame.display.update()
            #--------------------------------------------------------------------------
           
            #NORMALIZE EVERYONE'S WEIGHTS ----------------------------
            total_weight=0
            for j in range (numParticles):   
               total_weight+=self.particles[j].weight

            for j in range (len(sampling_list)):   
               sampling_list[j].weight=sampling_list[j].weight/total_weight
               #print "norm:",j, sampling_list[j].weight
                           
            #resample
            weights_interval=[]
            weight_=0
            resample_particles=[]

            #get the weights interval
            if(total_weight!=0 and i!=(len(self.observations)-1)):
               for j in range (len(sampling_list)):
                  weights_interval.append(weight_+sampling_list[j].weight)
                  weight_=weight_+sampling_list[j].weight
                  #print j, weights_interval[j]

               #get the resampled particles
               for k in range (numParticles):
                  weight_threshold=random.random()
                  sample=-1
                  for w in range (len(weights_interval)):
                     if (weight_threshold<=weights_interval[w] and sample==-1):
                        sample=w
                        #print "rand",weight_threshold,sample
              
                  #copy sampled particles
                  self.particles[k].copy_particle(sampling_list[sample],k)
                     
         done=True
      #  exit.
      pygame.quit ()
               
      #look through intervals
      self.get_avg_distance(ground_truth)

            
      #visualize particles
      self.visualize(landmarks_list,linePoints)


   def visualize(self, landmarks_list,linePoints):
      print "Visualizing..."
      
      #last normalization
      total_weight=0
      for j in range (len(self.particles)):   
         total_weight+=self.particles[j].weight
      
      for j in range (len(self.particles)):   
         if total_weight!=0:
            self.particles[j].weight=self.particles[j].weight/total_weight
            #print "norm:",j, self.particles[j].weight
      
      #get the particle with highest weight
      max_weight=self.particles[0].weight
      max_particle=0
      for i in range(len(self.particles)):
         if(self.particles[i].weight>max_weight):
            max_weight=self.particles[i].weight
            max_particle=i
      
      print "The particle with the highest is",self.particles[max_particle].index, "with",max_weight,"and average distance of",self.particles[max_particle].avg_difference
      
      list_colors=[BLUE, GREEN, PINK, GRAY, LBLUE]
      
      #Loop until the user clicks the close button ----------------------------------
      size = [500,500]
      screen = pygame.display.set_mode(size)
      pygame.display.set_caption("Path Visualization")
      done = False
      
      # Used to manage how fast the screen updates
      clock = pygame.time.Clock()

      state_num=0
      while done==False:
         for event in pygame.event.get(): # User did something
            if event.type == pygame.QUIT: # If user clicked close
               done=True # Flag that we are done so we exit this loop
         
         # Set the screen background
         screen.fill(WHITE)
         

      
         # Draw line
         path=copy.deepcopy(self.particles[max_particle].path.stateList)
         sqrColor=list_colors[0]
         for j in range(len(path)):
            current_state=copy.deepcopy(path[j])
            pygame.draw.rect(screen,sqrColor,[current_state.x,current_state.y,1,1], 2)

         # Draw line
         if len(linePoints) > 1:
            # draw lines (surface/screen, color of lines, closed/connect to first point, points to connect, width/thickness of lines)
            pygame.draw.lines( screen, RED, False, linePoints, 2 )      

         
         for i in landmarks_list:
            pos=i.get_position()
            sqrColor=BLACK
            pygame.draw.rect(screen,sqrColor,[pos[0],pos[1],1,1], 2)
         
         
         clock.tick(1)
         # Go ahead and update the screen with what we've drawn.
         pygame.display.flip()
      
      #  exit.
      pygame.quit ()


