# Authors: Tina Nye & Qandeel Sajid
# Project: AI-Project 2, Part A

import math
import sys
import random


###### Print Intro
print "\n--------------------------------"
print "Program: Bayesian Networks - Part A"
print "By: Tina Nye & Qandeel Sajid"

print " Varibales"
print "-----------\n"

print "OC: card holder owns a computer or smart phone."
print "Fraud: current transaction is fraudulent."
print "Trav: card holder is currently traveling."
print "FP: current transaction is a foreign purchase."
print "IP: current purchase is an internet purchase."
print "CRP: a computer related purchase was made in the past week."

## --FUNCTIONS
#functions return [false, true]
def getTrav():
   return [.95, .05]

def getOC():
   return [.4, .6]

def getFraud(trav):
   if(trav!=-1):
      if(trav==1):
         return [.99, .01]
      else:
         return [.996,.004]

def getCRP(oc):
   if(oc!=-1):
      if(oc==1):
         return [.90, .1]
      else:
         return [.999, .001]

def getFP(trav, fraud):
   if(trav!=-1 and fraud!=-1):
      if(trav==1 and fraud==1):
         return [ .1,.90]
      elif(fraud==0 and trav==1 ):
         return [.1, .90]
      elif(fraud==1 and trav==0):
         return [.9, .1]
      else:
         return [.99, .01]

def getIP(oc, fraud):
   if(oc!=-1 and fraud!=-1):
      if(oc==1 and fraud==1):
         return [.98,.02]
      elif(oc==1 and fraud==0):
         return [.99,.01]
      elif(oc==0 and fraud==1):
         return [.989,.011]
      else:
         return [.999, .001]

def likelihood(samples, allVariable, specificEvent, onlyEvidence):
   totalWeight=0
   subWeight=0
   
   #do likelihood weighting
   for i in range (samples):
      sample=["=", "=", "=", "=", "=", "="]
      weight=1
      
      #get travel
      temp=getTrav()
      if(onlyEvidence[0]=="="):
         rand=random.random()
         if(rand<temp[0]):
            sample[0]=0
         else:
            sample[0]=1
      else:
         sample[0]=onlyEvidence[0]
         weight=weight*temp[onlyEvidence[0]]
      
      #get OC
      temp=getOC()
      if(onlyEvidence[1]=="="):
         rand=random.random()
         
         if(rand<temp[0]):
            sample[1]=0
         else:
            sample[1]=1
      
      else:
         sample[1]=onlyEvidence[1]
         weight=weight*temp[onlyEvidence[1]]
      
      #get Fraud
      temp=getFraud(sample[0])    
      if(onlyEvidence[2]=="="):
         rand=random.random()
         
         if(rand<temp[0]):
            sample[2]=0
         else:
            sample[2]=1
      else:
         sample[2]=onlyEvidence[2]
         weight=weight*temp[onlyEvidence[2]]
      
      #get CRP
      temp=getCRP(sample[1])
      if(onlyEvidence[3]=="="):
         rand=random.random()
         if(rand<temp[0]):
            sample[3]=0
         else:
            sample[3]=1
      else:
         sample[3]=onlyEvidence[3]
         weight=weight*temp[onlyEvidence[3]]
      
      temp=getFP(sample[0], sample[2])
      if(onlyEvidence[4]=="="):
         rand=random.random()
         if(rand<temp[0]):
            sample[4]=0
         else:
            sample[4]=1
      else:
         sample[4]=onlyEvidence[4]
         weight=weight*temp[onlyEvidence[4]]
      
      
      temp=getIP(sample[1], sample[2])
      if(onlyEvidence[5]=="="):
         rand=random.random()
         if(rand<temp[0]):
            sample[5]=0
         else:
            sample[5]=1
      else:
         sample[5]=onlyEvidence[5]
         weight=weight*temp[onlyEvidence[5]]
      
      same=True
      for j in range (len(sample)):
         if(sample[j]!=specificEvent[j] and specificEvent[j]!=-1):
            same=False
      
      #get new weights
      if(same==True):
         subWeight=subWeight+weight
      totalWeight=totalWeight+weight
   
   print "\nProbability: " + str(float(subWeight)/float(totalWeight)) + " with " + str(samples) + " samples"


#get queries variables
queries=[]
evidence=[]
new=""
print "\nEnter the queries variables one by one. To insert the negation of a queries variable place '~' before the variable (e.g. ~Fraud). When complete, enter 'q'. \n"

while(new!="Q" and new!="q"):
   new=raw_input("queries: ")
   if(new!="Q" and new!="q"):
      queries.append(new)


print "\nEnter the evidence variables one by one.\n"

new=""
while(new!="Q" and new!="q"):
   new=raw_input("Evidence: ")
   if(new!="Q" and new!="q"):
      evidence.append(new)

#make the wanted atomic event, if True insert T, if F insert F, else insert -
allVariable=["Trav", "OC", "Fraud", "CRP", "FP", "IP"]
#0=Trav
#1=OC
#2=Fraud
#3=CRP
#4=FP
#5=IP

specificEvent=["=", "=", "=", "=", "=", "="]

#check if a variable if found in query
for i in range (len(allVariable)):
   find="IDK"
   for j in range(len(queries)):
      
      var=queries[j]
      seperate=var.split('~')
      
      if(var==str(allVariable[i])):
         find=1
      
      elif(len(seperate)==2 and seperate[0]=="" and seperate[1]==str(allVariable[i])):
         find=0
   
   if(find==1):
      specificEvent[i]=1
   
   elif(find==0):
      specificEvent[i]=0
   
   else:
      specificEvent[i]=-1


#check if a variable if found in evidence
onlyEvidence=["=", "=", "=", "=", "=", "="]
for i in range (len(allVariable)):
   find="IDK"
   for j in range(len(evidence)):
      
      var=evidence[j]
      seperate=var.split('~')
      
      if(var==str(allVariable[i])):
         find=1
      
      elif(len(seperate)==2 and seperate[0]=="" and seperate[1]==str(allVariable[i])):
         find=0
   
   if(find==1):
      specificEvent[i]=1
      onlyEvidence[i]=1
   
   elif(find==0):
      specificEvent[i]=0
      onlyEvidence[i]=0
   
   elif(specificEvent[i]==-1):
      specificEvent[i]=-1

#print specificEvent
#print onlyEvidence

#call likelihood weighting with varying samples values
samples=100
likelihood(samples, allVariable, specificEvent, onlyEvidence)

samples=1000
likelihood(samples, allVariable, specificEvent, onlyEvidence)

samples=10000
likelihood(samples, allVariable, specificEvent, onlyEvidence)

samples=100000
likelihood(samples, allVariable, specificEvent, onlyEvidence)

samples=1000000
likelihood(samples, allVariable, specificEvent, onlyEvidence)








