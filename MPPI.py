#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud
from tf.transformations import euler_from_quaternion
import numpy as np
import math
#state is [x,y,heading]
#control Action is [velocity, steer]


#Adham

#MPPI paramters
DT=1
HOR=20
SAMPLES=500
LAMBDA=1
###
#Vehilce Parameters
L=1
MAXSTEER=30
MAXVEL=10
## global variables
init_state=np.array([0,0,np.pi/2])
nom_U=np.zeros((HOR,2))
obstacles=[]
vel=0
error_integral=0
error_old=0
#callbacks for coppelia
##PID for coppelia
KP = 0.018
KI = 0.0812
KD = 0.0
def PID(target_vel:float):
    global vel
    global error_integral
    global error_old
    error = target_vel-vel
    error_integral=error_integral+error*DT
    error_differential=(error-error_old)/DT
    throttle=KP*error+KI*error_integral+KD*error_differential
    error_old=error
    if throttle < -0.1:
        brake = 0.7 * abs(throttle)
    else:
        brake=0.0
    throttle=max(0.0,min(throttle,1))
    brake=max(0.0,min(brake,1))
    return throttle,brake
def odom_callback(state:Odometry):
    global init_state,vel
    init_state[0]=state.pose.pose.position.x
    init_state[1]=state.pose.pose.position.y
    ori_w = state.pose.pose.orientation.w
    ori_x = state.pose.pose.orientation.x
    ori_y = state.pose.pose.orientation.y
    ori_z = state.pose.pose.orientation.z
    vel= math.hypot(state.twist.twist.linear.x,state.twist.twist.linear.y)
    roll, pitch, yaw = euler_from_quaternion([ori_x, ori_y, ori_z, ori_w])
    yaw += (math.pi / 2.0) ##for interfacing with Coppelia
    init_state[2]=yaw
def obstacle_callback(msg:PointCloud):
    global obstacles
    obstacles=msg.points

##
##MPPI Functions
def forwardModel(x0, u):
    u=np.clip(u,[0,-1*MAXSTEER],[MAXVEL,MAXSTEER])
    beta=u[1]/2*(np.pi/180)
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
        for obs in obstacles:
            if math.hypot(state[0]-obs[0],state[1]-obs[1])<obs[2]:
                return np.inf
    cost = math.hypot(states[HOR,0]-init_state[0],states[HOR,1]-init_state[1])
    ##to be edited later
    '''
        stateCost=(state[1]-state[0])+5
        cost+=stateCost
        ###
        '''
    return cost

def generateControlActions():
    perturbations=np.random.rand(HOR,2)-np.full((HOR,2),[0,0.5])
    scale=np.array([[MAXVEL,0],[0,2*MAXSTEER]])
    Du=perturbations@scale
    return Du
def FindOptimalControlActions():
    global nom_U
    costs = np.zeros(SAMPLES)
    Dus=np.zeros((SAMPLES,HOR,2))
    for i in range(SAMPLES):
        Dus[i]=generateControlActions()
        costs[i]=calculateCost(nom_U+Dus[i])
    #print("costs = "+str(costs))
    weights=np.exp(-(1/LAMBDA)*costs)
    #print("weights = "+str(weights))
    Du=np.einsum("i,ijk->jk",weights,Dus)*(1/(weights.sum(0)))
    print("Du ="+ str(Du))
    print("######")
    nom_U=nom_U+Du
    nom_U=np.clip(nom_U,[0,-1*MAXSTEER],[MAXVEL,MAXSTEER])
    return nom_U
    '''Du_2=np.zeros((HOR,2))
    for i in range(SAMPLES):
        Du_2+=Dus[i]*weights[i]
    print("Final perturbation = "+str(Du))
    print("Final perturbation 2 = "+str(Du_2))'''
###
##Subscribers and publishers for Coppelia
brakes_pub = rospy.Publisher('/brakes', Float64, queue_size=10)
state_sub=rospy.Subscriber('/odom',Odometry,callback=odom_callback,queue_size=10)
throttle_pub=rospy.Publisher('/cmd_vel',Float64,queue_size=10)
steer_pub=rospy.Publisher('/SteeringAngle',Float64,queue_size=10)
obstacle_sub=rospy.Subscriber("/boundary_points", PointCloud, callback=obstacle_callback)
##

if __name__ == '__main__' :
    rospy.init_node("MPPI")
    rate=rospy.Rate(1/DT)
    while not rospy.is_shutdown():
        FindOptimalControlActions()
        throttle,brake = PID(nom_U[0,0])
        #print(nom_U)
        steer= nom_U[0,1]
        print("target velocity ="+str(nom_U[0,0])+", throttle ="+ str(throttle)+", brakes ="+str(brake)+", steer = "+str(steer))
        brakes_pub.publish(Float64(brake))
        throttle_pub.publish(Float64(throttle))
        steer_pub.publish(Float64(steer))
        rate.sleep()