import random, os
import pygame
import math
import vector
import robot
import state
import observation
import gaussian
import particle_filter

pygame.init()



# Define Position of Robot
robX = 10
robY = 10

# Define Speed
linVelocity = 2
rotVelocity = 1

# Direction Of Robot
robDirection = 0

#global bools
MAP_FILE=False
PATH_FILE=False


# Set initial mouse position
pygame.mouse.set_pos([robX, robY])
mousePosChange = pygame.mouse.get_rel()

# Define some colors
BLACK = ( 0, 0, 0)
WHITE = ( 255, 255, 255)
GREEN = ( 0, 255, 0)
RED = ( 255, 0, 0)
BLUE=(0,0,255)
GRAY=(88, 88, 88)
PINK=(255,0,255)
LBLUE=(0,255,255)

#FUNCTION DEFINITION =============================================================
#==================================================================================
def get_angle(current, next):
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

def angular_difference(firstA, secondA):
   difference = secondA - firstA;
   while (difference < -180): 
      difference += 360;
   while (difference > 180): 
      difference -= 360;
   return difference

def angular_summation(angle, difference):
   ang=math.degrees(angle)
   while(difference < 0): 
      difference += 360;
      #print "+", difference
   while (difference > 0): 
      difference -= 360;
      #print "-", difference
   sum = -difference+ang;
   if(sum>360):
      sum-=360
   return sum

def get_distance(position, target):
   return math.sqrt(((position[0]-target[0])*(position[0]-target[0]))+((position[1]-target[1])*(position[1]-target[1])))

def observation_model(path,file_num, vis_range, noise_num, mean, std_v,std_v_rot):
   #get displacement and rotation
   observations=[]
   orig=[10,10]
   orig_rot=0
   whole_path=path.get_Path()
   
   #make observations from the path
   for i in range(len(whole_path)):
      #variables
      observe=observation.Observation()
      disp=0
      rot=0
      current_pos=[whole_path[i].x, whole_path[i].y]
      current_rot=whole_path[i].theta
      
      #check if it's the first value
      if(i==0):
         disp=math.sqrt(((orig[0]-current_pos[0])*(orig[0]-current_pos[0]))+((orig[1]-current_pos[1])*(orig[1]-current_pos[1])))+gaussian.main(mean,std_v)
         rot=angular_difference(current_rot, orig_rot)+math.degrees(gaussian.main(mean,std_v_rot))
      else:
         #get displacement
         orig=[whole_path[i-1].x, whole_path[i-1].y]
         disp=math.sqrt(((orig[0]-current_pos[0])*(orig[0]-current_pos[0]))+((orig[1]-current_pos[1])*(orig[1]-current_pos[1])))+gaussian.main(mean,std_v)
         
         #get angular difference
         orig_rot=whole_path[i-1].theta
         rot=angular_difference(current_rot, orig_rot)+math.degrees(gaussian.main(mean,std_v_rot))
      
      #get visible landmarks
      for j in landmarks_list:
         land_pos=j.get_position()
         land_dist=get_distance(current_pos, land_pos)
         
         #treshold landmarks
         if(land_dist<vis_range):
            land_id=j.get_id()
            land_angle=math.radians(get_angle(current_pos, land_pos))
            observe.append_landmark(land_id, (land_angle-math.radians(current_rot)))
            #print "current pos" + str(current_pos)+" landmark("+str(j.get_id())+")_pos: " +str(land_pos) + " dist: " + str(land_dist)           
            #print "current angle: " + str(current_rot)+" landmark("+str(j.get_id())+")_angle: " +str(land_angle) + " dist: " + str((land_angle-math.radians(current_rot)))
      
      #assign new observations
      observe.disp=disp
      observe.rot=rot
      observe.numLandmarks=len(observe.landmarkPair)
      observations.append(observe)
   
   #get map file name -------------------------------------------------
   sensing_name="output/sensing_"+str(file_num)+"_"+str(noise_num)+".txt"
   
   #try to open the file
   fout=open(sensing_name, "w")
   
   fout.write(str(len(observations)))
   for i in observations:
      #fout.write("\n"+ str(i.disp) + " "+ str(i.rot)+" "+str(i.numLandmarks)+" "+str(i.landmarkPair))
      fout.write("\n")
      i.output(fout)
   fout.close()



def path_reconstruction(file_num, noise_num, list_diff, path):
   #get map file name -------------------------------------------------
   ground_truth=path.get_Path()
   sensing_name="output/sensing_"+str(file_num)+"_"+str(noise_num)+".txt"
   avg_dist=0
   avg_theta=0
   
   #try to open the file
   fin=open(sensing_name, "r")
   path_length=int(fin.readline())
   
   previous_pos=[10.0,10.0]
   previous_angle=math.radians(0.0)
   linePoints=[]
   
   #iterate through path
   for i in range(path_length):
      pos=fin.readline()
      pos=pos.split(' ')
      
      cos_angle=math.cos(previous_angle)
      sin_angle=-math.sin(previous_angle)
      diff=float(pos[0])
      
      x_i_plus1=previous_pos[0]+(diff*cos_angle)
      y_i_plus1=previous_pos[1]+(diff*sin_angle)
      
      angle_i=angular_summation(previous_angle, float(pos[1]))
   
      #get difference
      if i==0:
         ground_diff=math.sqrt(((ground_truth[i].x-previous_pos[0])*(ground_truth[i].x-previous_pos[0]))+((ground_truth[i].y-previous_pos[1])*(ground_truth[i].y-previous_pos[1])))
         sense_diff=math.sqrt(((x_i_plus1-previous_pos[0])*(x_i_plus1-previous_pos[0]))+((y_i_plus1-previous_pos[1])*(y_i_plus1-previous_pos[1])))
         avg_dist+=(ground_diff-sense_diff)
         avg_theta+=(ground_truth[i].theta-angle_i)
      else:
         ground_diff=math.sqrt(((ground_truth[i].x-ground_truth[i-1].x)*(ground_truth[i].x-ground_truth[i-1].x))+((ground_truth[i].y-ground_truth[i-1].y)*(ground_truth[i].y-ground_truth[i-1].y)))
         sense_diff=math.sqrt(((x_i_plus1-previous_pos[0])*(x_i_plus1-previous_pos[0]))+((y_i_plus1-previous_pos[1])*(y_i_plus1-previous_pos[1])))
         avg_dist+=math.fabs(ground_diff-sense_diff)
         avg_theta+=math.fabs(ground_truth[i].theta-angle_i)

      previous_pos=[x_i_plus1,y_i_plus1]
      previous_angle=math.radians(angle_i)
      #print str(x_i_plus1)+ " "+ str(y_i_plus1)+ " "+str(angle_i)
      linePoints.append([x_i_plus1, y_i_plus1])

   
   #aveage the values
   avg_dist/=path_length
   avg_theta/=path_length
   list_diff.append([avg_dist,avg_theta])
            
   fin.close()
   return linePoints




#==================================================================================
#==================================================================================
#==================================================================================




#get # of landmarks
landmarks=raw_input("Enter the number of landmarks or default to 40 by leaving it blank: ")
if(landmarks==""):
   landmarks=40
landmarks=int(landmarks)
landmarks_list=[]


#get map file name -------------------------------------------------
#map_name=raw_input("Enter the file name for map file or just leave it blank: ")
file_num=raw_input("Enter number of the files: ")
map_name="output/map_"+str(file_num)+".txt"

#try to open the file
try:
   fout=open(map_name, "r")
except IOError:
   MAP_FILE=False
else:
   MAP_FILE=True

#if file does open, make random landmarks
if (MAP_FILE==False):
   #print "make random landmarks: " + str(landmarks)
   i=0
   while(len(landmarks_list)!=landmarks):
      x=int(math.floor(random.random()*500))
      y=int(math.floor(random.random()*500))
      if(landmarks_list.count([x,y])==0 and x>=0 and x<=500 and y>=0 and y<=500):
         mark=state.Landmark(i, [x,y])
         landmarks_list.append(mark)
         i+=1

#read landmarks from file
else:
   print "\nReading map file\n"
   landmarks=int(fout.readline())
   for i in range (landmarks):
      pos=fout.readline()
      pos=pos.split(' ')
      mark=state.Landmark(i, [int(pos[0]),int(pos[1])])
      landmarks_list.append(mark)
   fout.close()
#--------------------------------------------------------------




#-----------------------------------------------------------------------------------
#------------------------------------------------------------------------------------
#-- print out the landmarks ------------
if(MAP_FILE==False):
   fout=open("output/map.txt", "w")
   fout.write(str(len(landmarks_list)))
   for i in landmarks_list:
      pos=i.get_position()
      fout.write("\n"+str(pos[0]) + " " + str(pos[1]))
   fout.close()

   print "map.txt outputted\n"

#-----Ask for path file ---------------
#path_name=raw_input("Enter the file name for path file or just leave it blank: ")
path_name="output/path_"+str(file_num)+".txt"

#try to open the file
try:
   fout=open(path_name, "r")
except IOError:
   PATH_FILE=False
else:
   PATH_FILE=True

#define linePoints and path variables
linePoints = []   # list for points
path=state.Path()

#get input from the file
if (PATH_FILE==True):
   print "\nReading path file\n"
   path_length=int(fout.readline())
   for i in range (path_length):
      pos=fout.readline()
      pos=pos.split(' ')
      state_=state.State(float(pos[0]),float(pos[1]), float(pos[2]))
      path.append_state(state_)
      linePoints.append([float(pos[0]), float(pos[1])])
   fout.close()
      
else:
   linePoints.append([10,10])

#=====================================================================================
#ask fro path input in sensing file are not provided =================================
#=====================================================================================
if PATH_FILE==False:
   
   # Set the height and width of the screen and bool
   size = [500,500]
   screen = pygame.display.set_mode(size)
   pygame.display.set_caption("Path Visualization")
   done = False

   # Used to manage how fast the screen updates
   clock = pygame.time.Clock()

   lineColor = RED   # color of lines
   sprite = robot.Sprite()   # create the sprite
   
   rot_finished=True
   angle_target=0

   #Loop until the user clicks the close button ----------------------------------
   while done==False:
      for event in pygame.event.get(): # User did something
         if event.type == pygame.QUIT: # If user clicked close
            done=True # Flag that we are done so we exit this loop
         elif (event.type == pygame.MOUSEBUTTONDOWN) and PATH_FILE==False:
               print "Left mouse clicked"

               # assign the sprite target to the mouse click position
               sprite.target = event.pos

               # add the mouse click position to the line list
               linePoints.append(event.pos)

               rot_finished=False

      #do rotation ---------------------------------------------------------------
      if(sprite.get_target()!=None and rot_finished==False and PATH_FILE==False):
         angle_target=get_angle(sprite.get_current_loc(),sprite.get_target())
         if(sprite.get_angle()==angle_target):
            #print "rotation finished"
            rot_finished=True
         else:
            #print "current: " + str(sprite.get_angle()) + " target: " + str(angle_target)
            change=sprite.get_angle()-angle_target
            
            #if angle difference more than 180
            if(math.fabs(change)>180):
               
               #current angle less than target
               if(change<0):
                  #print ">180 & change<0"
                  diff=360+change
                  if(diff>math.degrees(rotVelocity)):
                     increment=math.degrees(rotVelocity)
                  else:
                     increment=diff
               
                  #get new angle
                  angle=sprite.get_angle()-increment
                  if(angle<=0):
                     angle+=360
               else:
                  #print "<180 & change>0"
                  diff=360-change
                  if(diff>math.degrees(rotVelocity)):
                     increment=math.degrees(rotVelocity)
                  else:
                     increment=diff
                  
                  #get new angle
                  angle=sprite.get_angle()+increment
                  if(angle>360):
                     angle=angle-360
                        
               #rotate
               sprite.image = pygame.transform.rotate(sprite.get_original(), angle)
               path.append_state(sprite.set_angle(angle))

            #the difference if less than 180
            else:
               #decide increment value
               diff=math.fabs(change)
               if(diff>math.degrees(rotVelocity)):
                  diff=math.degrees(rotVelocity)
               
               #current<target angle
               if(change<0):
                  angle=sprite.get_angle()+diff
                  if(angle>=360):
                     angle=angle-360
               else:
                  angle=sprite.get_angle()-diff
                  if(angle<0):
                     angle+=360
                        
               #rotate
               sprite.image = pygame.transform.rotate(sprite.get_original(), angle)
               path.append_state(sprite.set_angle(angle))
      #------------------------------------------------------------------------------
            

      # Set the screen background
      screen.fill(WHITE)


      # update the sprite
      if rot_finished==True and PATH_FILE==False:
         path.append_state(sprite.update())

      
      # Draw Robot To Screen
      screen.blit( sprite.image, sprite.rect.topleft )


      # Draw line
      if len(linePoints) > 1:
         # draw lines (surface/screen, color of lines, closed/connect to first point, points to connect, width/thickness of lines)
         pygame.draw.lines( screen, lineColor, False, linePoints, 2 )
            
      for i in landmarks_list:
         pos=i.get_position()
         sqrColor=BLACK
         pygame.draw.rect(screen,sqrColor,[pos[0],pos[1],1,1], 2)

      # Limit frames per second by the velocity
      if(rot_finished==True):
         clock.tick(linVelocity*10)
      else:
         clock.tick(linVelocity*2)
      # Go ahead and update the screen with what we've drawn.
      pygame.display.flip()

   #  exit.
   pygame.quit ()

#-- print out the path ------------
if(PATH_FILE==False):
   path.output_list();
else:
   sensing_finished=raw_input("Are the sensing files complete ('Y' or 'N'): ")
      
   if(sensing_finished=='N'):
      #get visibility range
      vis_range=40
      vis_range=raw_input("Enter the visibility range or leave it blank to default to 40: ")
      if(vis_range!=""):
         vis_range=int(vis_range)

      mean=0
      std_v=0
      std_v_rot=0
      observation_model(path,file_num, vis_range,1, mean, std_v,std_v_rot)

      
      mean=0
      std_v=.25
      std_v_rot=0
      observation_model(path,file_num, vis_range,2, mean, std_v,std_v_rot)

      mean=0
      std_v=0
      std_v_rot=.01
      observation_model(path,file_num, vis_range,3,mean, std_v,std_v_rot)

      mean=0
      std_v=.25
      std_v_rot=.01
      observation_model(path,file_num, vis_range,4,mean, std_v,std_v_rot)

      mean=0
      std_v=.5
      std_v_rot=.03
      observation_model(path,file_num, vis_range,5,mean, std_v,std_v_rot)

   else:
      use_filter=raw_input("To visualize the sensed paths enter 'V' else enter nothing to use the particle filter: ")
      
      if(use_filter=='V' or use_filter=='v'):
         list_sensed_paths=[]
         list_diff=[]
         list_sensed_paths.append(linePoints)
         for i in range(5):
            paths=path_reconstruction(file_num,i+1, list_diff, path) #calculated average error in the function
            list_sensed_paths.append(paths)

         list_colors=[RED, BLUE, GREEN, PINK, GRAY, LBLUE]
         
         #Loop until the user clicks the close button ----------------------------------
         size = [500,500]
         screen = pygame.display.set_mode(size)
         pygame.display.set_caption("Path Visualization")
         done = False

         # Used to manage how fast the screen updates
         clock = pygame.time.Clock()
         while done==False:
            for event in pygame.event.get(): # User did something
               if event.type == pygame.QUIT: # If user clicked close
                  done=True # Flag that we are done so we exit this loop
            
            # Set the screen background
            screen.fill(WHITE)
            
            # Draw line
            for j in range(len(list_sensed_paths)):
                  pygame.draw.lines( screen, list_colors[j], False, list_sensed_paths[j], 2 )
            
            for i in landmarks_list:
               pos=i.get_position()
               sqrColor=BLACK
               pygame.draw.rect(screen,sqrColor,[pos[0],pos[1],1,1], 2)
            

            clock.tick(linVelocity*2)
            # Go ahead and update the screen with what we've drawn.
            pygame.display.flip()

         #  exit.
         pygame.quit ()

         #print out average distances
         count=1
         for j in list_diff:
            print "Sensing " + str(count) + " Average Distance ERROR: " +str(j[0])+ " Average Orientation ERROR: " + str(j[1])
            count+=1

      else:
         sensed_file=raw_input("Enter the sensing number (1-5): ")
         particles=raw_input("Enter the number of particles: ")
         
         #call the filter
         filter=particle_filter.Particle_Filter()
         filter.algorithm(int(particles), sensed_file, file_num,landmarks_list,linePoints, path)
         



