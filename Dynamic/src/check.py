#!/usr/bin/env python
from __future__ import division
import math 
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import numpy as np
import time
import matplotlib.pyplot as plt 
np.seterr(divide='ignore', invalid='ignore')
from tf.transformations import euler_from_quaternion, quaternion_from_euler
global xgps, ygps, get_time, roll, pitch, yaw, xact, yact, vact, wact

#***************1st *****************
K_x, K_y, K_theta = 0.05, 0.0025, 0.025
#***************2nd******************
c1, c2, c3 = 2, 0.01, 1 
#************************************
K_I, K_TH = 1, 5
K_TH = 5
#******3rd Law From Lee's Paper *****
K_X, K_Y, K_THETA = None, None, None 

mengx = np.zeros([3,1])
mengy = np.zeros([3,1])
#********************************************** Menger Curve Fuction *****************************************************************************************************
def menger(XM,YM):
    f = 0
    area = abs(0.5*(XM[f,0]*(YM[f+1,0] - YM[f+2,0]) + XM[f+1,0]*(YM[f+2,0] - YM[f,0]) + XM[f+2,0]*(YM[f,0]-YM[f+1,0])))
    s1= math.sqrt(pow((XM[f,0]-XM[f+1,0]),2)+ pow((YM[f,0] - YM[f+1,0]),2))
    s2= math.sqrt(pow((XM[f+2,0]-XM[f+1,0]),2) + pow((YM[f+2,0] - YM[f+1,0]),2))
    s3= math.sqrt(pow((XM[f,0]-XM[f+2,0]),2) + pow((YM[f,0] - YM[f+2,0]),2))
    k= 4*area/(s1*s2*s3)
    return k
#********************************************** GPS Callback Function ****************************************************************************************************
def datagps(data):
    global roll , pitch , yaw ,xact , yact, vact , wact
    roll, pitch, yaw, xact, yact = 0.0, 0.0, 0.0, 0.0, 0.0
    orintation_q = data.pose.pose.orientation
    orientation_list = [orintation_q.x, orintation_q.y,orintation_q.z,orintation_q.w]
    (roll , pitch, yaw) = euler_from_quaternion(orientation_list)
    xact =  data.pose.pose.position.x
    yact = data.pose.pose.position.y

#*************************************************************************************************************************************************************************
if __name__ == "__main__":
    global xact, yact, yaw 
    xact, yact, yaw  = 20, 0.0, 0.0 
    vref, radius = 8, 20
    omega_ref_old = vref/radius 
    xe, ye, theta_e = 0.0, 0.0, 0.0
    x_refold, y_refold = 20.0 , 0.0
    theta, deltat = 0.0, 0.0

    rospy.init_node('check',anonymous=True) 
    rospy.Subscriber('/agent_0/odom', Odometry, datagps)
    pub_speed = rospy.Publisher('/agent_0/cmd_vel',Twist,queue_size=10)
    speed_car = Twist()
    time_init = rospy.get_time()
    # time.sleep(5)
    # speed_car.linear.x = vref
    # speed_car.angular.z = omega_ref_old
    # pub_speed.publish(speed_car) 

    # time.sleep(0.032)
    while not rospy.is_shutdown():
        time_curr = rospy.get_time()
        deltat = time_curr - time_init     
        global xact, yact, yaw
        #**************************************************
        theta_new = theta + deltat*(omega_ref_old)
        x_refnew = radius*math.cos(theta_new)
        y_refnew = radius*math.sin(theta_new) 
        #**************************************************
        # x_new = x_init + vref*(time_curr - time_init)       
        # y_new = y_init
        #*******************menger curve part ************** 
        # g=0
        # mengx[g+2,0] = mengx[g+1,0]
        # mengx[g+1,0] = mengx[g,0]
        # mengy[g+2,0] = mengy[g+1,0]
        # mengy[g+1,0] = mengy[g,0]                
        # mengx[g,0] = x_new
        # mengy[g,0] = y_new
        # kk = menger(mengx,mengy)  
        # **************************************************             
        # theta_ref_new = (-math.pi)/2
        theta_act = yaw
        if theta_act< 0:
            theta_act += 2*math.pi                                                                       # Actual angle 
        
        theta_ref_new = math.atan2( (y_refnew - y_refold), (x_refnew - x_refold) )     # Ref angle         

        if theta_ref_new<0:
            theta_ref_new += 2*math.pi

        #****************************************************************************************
        xe = float(math.cos(theta_act)*(x_refnew - xact) + math.sin(theta_act)*(y_refnew - yact))        
        ye = float(-(math.sin(theta_act)*(x_refnew - xact)) + math.cos(theta_act)*(y_refnew - yact))
        theta_e = theta_ref_new - theta_act
        print x_refnew, y_refnew, xact, yact , xe, ye, theta_e, time_curr 
        # print "******************************************"
        #****************************************************************************************
        # control_velocity = vref*math.cos(theta_e) + K_x*xe #1st
        control_velocity = (vref) + ((c1*xe)/(math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2))))) #2nd
        
        # control_velocity = vref*math.cos( theta_e) + K_X*xe # 3rd law from Lee's paper
        
        # controlled_omega = omega_ref_old + (vref)*(K_y*ye + K_theta*math.sin(theta_e)) #1st
        controlled_omega = omega_ref_old + ( ((c2*vref)*(ye*math.cos(theta_e/2) - xe*math.sin(theta_e/2) ))/( math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))) ) + c3*math.sin(theta_e/2) #2nd 
        
        # controlled_omega = omega_ref_old + K_Y*vref*ye + K_THETA*math.sin(theta_e) # 3rd law from Lee's paper
        #****************************************************************************************       
        speed_car.linear.x = control_velocity
        speed_car.angular.z = controlled_omega
        pub_speed.publish(speed_car)  
        y_refold = y_refnew
        x_refold = x_refnew      
        theta = theta_new
        time_init = time_curr
        time.sleep(0.064)  
        