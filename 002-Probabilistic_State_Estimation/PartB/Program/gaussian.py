import math
import array
import random
import sys

RAND_MAX=2147483647


class Gaussian():
   def __init__(self):
      self.gauss_random = 0;
      self.available_gauss = 0;
      self.prev_mean = -100000;
      self.prev_variance = -10000;


def read_seed():
   fp = open("/dev/random", "rb");
   seed=fp.read(sys.getsizeof(int))
   fp.close()
   return seed

def generate_seed():
   seed=read_seed()
   random.seed(seed)
   return seed

def generate_seed_verbose():
   seed=generate_seed()
   fp.write ("seed: %x\n" % (seed))
   return seed

def uniform_random_():
   return (float(random.randint(0, RAND_MAX))/float(RAND_MAX))

def uniform_random(min, max):
   return ((uniform_random_()*(max-min))+min)
           
def uniform_int_random(min, max):
   return int(round(uniform_random(float(min)-0.499999, float(max)+0.499999)))

def gaussian_random(g, m, v):
   if (g.available_gauss==False or m!=g.prev_mean or v!=g.prev_variance):
      rsq=1.0
      rsq=0.0
      fac=0
      while(rsq>=1.0 or rsq==0.0):
         r1=uniform_random(-1,1)
         r2=uniform_random(-1, 1)
         rsq=r1*r1+r2*r2
                                 
      fac=math.sqrt((-2.0*math.log(rsq))/rsq)
      g.gauss_random=(r2*fac)*math.sqrt(v) + m;
      g.available_gauss = True;
      g.prev_mean = m;
      g.prev_variance = v;

      return (r1*fac)*math.sqrt(v) + m;

   else:
      g.available_gauss = False;
      return g.gauss_random;
   

def main(m,v):
   generate_seed()
   N=100
   sum=0
   gaussian=Gaussian()
   x=[]
   
   #print "before loop"
   for i in range (N):
      x.append(gaussian_random(gaussian, m, v))
      sum+=x[i]
   """
   for i in range (N):
      print x[i]
   """
   return (sum/float(N))

#main(0.0, 4.0)

