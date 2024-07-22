#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry, Path
import numpy as np
import math
#state is [x,y,heading]
#control Action is [velocity, steer]


#Adham 

#MPPI paramters
DT=0.1
HOR=15
SAMPLES=1000
###
#Vehilce Parameters
L=1
MAXSTEER=30
MAXVEL=10
##
init_state=np.zeros(3)
nom_U=np.zeros(HOR,2)
#path=
def forwardModel(x0, u):
    beta=u[1]/2
    x1=x0
    x1[0]=x1[0]+DT*u[0]*math.cos(x1[2]+beta)
    x1[1]=x1[1]+DT*u[0]*math.sin(x1[2]+beta)
    x1[2]=x1[2]+DT*(u[0]/L)*math.sin(beta)
    return x1
def calculateCost(Actions):
    states=np.full((HOR+1,3),init_state)
    for i in range(HOR):
        states[i+1]=forwardModel(states[i],Actions[i])
    cost = 0
    for state in states:
        ##to be edited later
        stateCost=state[0]-state[1]
        cost+=stateCost
        ###
    return cost
def generateControlActions():
    perturbations=np.random.rand(HOR,2)-np.full((HOR,2),0.5)
    scale=np.array([[2*MAXVEL,0],[0,2*MAXSTEER]])
    Du=perturbations@scale
    return Du
def FindOptimalControlActions():
    costs = np.zeros(SAMPLES)
    Dus=np.zeros(SAMPLES,HOR,2)
    for i in range(SAMPLES):
        Dus[i]=generateControlActions()
        costs[i]=calculateCost(nom_U+Dus[i])
    weights=np.exp(-1*costs)
if __name__ == '__main__' :
    rospy.init_node("sim")
    x0=np.array([0.0,0.0,0.0])
    u=np.array([1,0,0])
    states=np.array([[0.0,0.0,0.0],[1.0,0.0,0.0],[2.0,0.0,0.0],[3.0,0.0,0.0],[4.0,0.0,0.0]])
    u0=np.zeros((HOR,2))
    print(generateControlActions(u0))
    rospy.spin()