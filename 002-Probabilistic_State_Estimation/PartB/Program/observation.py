class Observation():

   def __init__(self):
      self.disp = 0
      self.rot = 0
      self.numLandmarks = 0
      self.landmarkPair = []

   def append_landmark(self, lId, angle):
      self.landmarkPair.append( [lId, angle] )

   def output(self, fout):
      fout.write(str(self.disp)+" "+str(self.rot) + " "+str(self.numLandmarks))
      for i in self.landmarkPair:
         fout.write(" "+str(i))



