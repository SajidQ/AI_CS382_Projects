import pygame
import vector
import state

class Sprite(pygame.sprite.Sprite):
   
   def __init__(self):
      self.image = pygame.image.load("Images/arrow.png").convert_alpha() # load image
      self.rect = self.image.get_rect()
      
      self.trueX = 10 # created because self.rect.center does not hold
      self.trueY = 10 # decimal values but these do
      self.rect.center = (self.trueX, self.trueY) # set starting position
      self.speed = 2 # movement speed of the sprite
      self.speedX = 0 # speed in x direction
      self.speedY = 0 # speed in y direction
      
      self.target = None # starts off with no target
      self.original=self.image
      self.angle=0
   
   def get_direction(self, target):
      '''
         Function:
         takes total distance from sprite.center
         to the sprites target
         (gets direction to move)
         Returns:
         a normalized vector
         Parameters:
         - self
         - target
         x,y coordinates of the sprites target
         can be any x,y coorinate pair in
         brackets [x,y]
         or parentheses (x,y)
         '''
      # if the square has a target
      if self.target:
         # create a vector from center x,y value
         position = vector.Vector(self.rect.centerx, self.rect.centery)
         
         # and one from the target x,y
         target = vector.Vector(target[0], target[1]) 
         
         # get total distance between target and position
         self.dist = target - position 
         
         # normalize so it's constant in all directions
         direction = self.dist.normalize() 
         return direction
   
   def distance_check(self, dist):
      '''
         Function:
         tests if the total distance from the
         sprite to the target is smaller than the
         ammount of distance that would be normal
         for the sprite to travel
         (this lets the sprite know if it needs
         to slow down. we want it to slow
         down before it gets to it's target)
         Returns:
         bool
         Parameters:
         - self
         - dist
         this is the total distance from the
         sprite to the target
         can be any x,y value pair in
         brackets [x,y]
         or parentheses (x,y)
         '''
      dist_x = dist[0] ** 2 # gets absolute value of the x distance
      dist_y = dist[1] ** 2 # gets absolute value of the y distance
      t_dist = dist_x + dist_y # gets total absolute value distance
      speed = self.speed ** 2 # gets aboslute value of the speed
      
      if t_dist < (speed): # read function description above
         return True
   
   
   def update(self):
      '''
         Function:
         gets direction to move then applies
         the distance to the sprite.center
         '''
      
      global robDirection
      self.dir = self.get_direction(self.target) # get direction
      # if there is a direction to move
      if self.dir:
         
         # if we need to stop
         if self.distance_check(self.dist): 
            # center the sprite on the target
            self.rect.center = self.target 
            return state.State( self.trueX, self.trueY, self.angle )
         
         else:
            # calculate speed from direction to move and speed constant
            self.trueX += (self.dir[0] * self.speed) 
            self.trueY += (self.dir[1] * self.speed)
            self.rect.center = (round(self.trueX),round(self.trueY)) # apply values to sprite.center
            return state.State( self.trueX, self.trueY, self.angle )
   
   def get_original(self):
      return self.original
   
   def get_target(self):
      return self.target
   
   def get_current_loc(self):
      return self.rect.center

   def set_angle(self, angle):
      self.angle=angle
      return state.State( self.trueX, self.trueY, self.angle )

   def get_angle(self):
      return self.angle



