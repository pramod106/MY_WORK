#!/usr/bin/env python
from __future__ import division
import math 
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
np.seterr(divide='ignore', invalid='ignore')
from tf.transformations import euler_from_quaternion, quaternion_from_euler
global xgps, ygps, get_time, roll, pitch, yaw, xact, yact, vact, wact 

c1, c2, c3 = 2, 0.01, 1.1 

#********************************************** GPS Callback Function ****************************************************************************************************
def datagps(data):
    global roll , pitch , yaw ,xact , yact, vact, wact
    roll, pitch, yaw, xact, yact = 0.0, 0.0, 0.0, 0.0, 0.0
    orintation_q = data.pose.pose.orientation
    orientation_list = [orintation_q.x, orintation_q.y,orintation_q.z,orintation_q.w]
    (roll , pitch, yaw) = euler_from_quaternion(orientation_list)
    xact =  data.pose.pose.position.x
    yact = data.pose.pose.position.y
    vact = data.twist.twist.linear.x
    wact = data.twist.twist.angular.z
    # print vact,wact
#*************************************************************************************************************************************************************************

if __name__ == "__main__":
    global xact, yact, yaw , vact, wact 
    xact, yact, yaw  = 0.0, 0.0, 0.0
    vact, wact = 0.0, 0.0 
    x_ref, y_ref = 0.0 , 0.0 
    vref, Amplitude, Lob = 5, 10, 40
    x_refold, y_refold, theta_old = 0.0, 0.0, 0.0
    theta_up = 0.0
    rospy.init_node('check',anonymous=True) 
    rospy.Subscriber('/agent_0/odom', Odometry, datagps)
    pub_speed = rospy.Publisher('/agent_0/cmd_vel',Twist,queue_size=10)
    pub_graph_data = rospy.Publisher('/graph_data', Float64MultiArray, queue_size= 100)
    graph_data = Float64MultiArray()
    # pub_act = rospy.Publisher('/pub_data', Pose, queue_size=10 )
    t = rospy.Rate(15.625) 
    speed_car = Twist()
    # act_data = Pose()
    time.sleep(5.0)
    time_init = rospy.get_time()

    while True:
        time_curr = rospy.get_time()
        deltat = time_curr - time_init
        theta_dot_up = vref/( math.sqrt( pow(Amplitude,2)*pow(math.cos(theta_up),2) + ( pow(Lob,2)/pow(math.pi,2) ) ))   # theta dot calculation
        theta_up = theta_old + deltat*theta_dot_up
        global xact, yact, yaw
        global vact, wact

        y_ref = Amplitude*math.sin(theta_up)
        x_ref = (Lob*theta_up)/(math.pi)
        
        theta_act = yaw
        if theta_act< 0:
            theta_act += 2*math.pi
        
        theta_ref_new = math.atan2( (y_ref - y_refold), (x_ref - x_refold) )
        if theta_ref_new<0:
            theta_ref_new += 2*math.pi
        # print theta_ref_new, theta_act
        #*************************************************************************************************
        xe = float(math.cos(theta_act)*(x_ref - xact) + math.sin(theta_act)*(y_ref - yact))        
        ye = float(-(math.sin(theta_act)*(x_ref - xact)) + math.cos(theta_act)*(y_ref - yact))
        theta_e = theta_ref_new - theta_act
        #*************************************************************************************************
        omega_ref_old = -(Amplitude*Lob*theta_dot_up*math.sin(theta_up)/ (math.pi*( ( pow(Lob,2)/pow(math.pi,2) ) + pow(Amplitude,2)*pow( math.cos(theta_up),2 ) )))
        
        graph_data.data = [x_ref, y_ref, xact, yact]                                     # Data Publish to plot
        
        pub_graph_data.publish(graph_data)
        
        if theta_e < -math.pi:
            theta_e+=2*math.pi
        elif theta_e > math.pi:
            theta_e-=2*math.pi 
        
        # print x_ref,y_ref,xact,yact,xe, ye, theta_e, time_curr                           # offline Plot
        
        control_velocity = (vref) + ((c1*xe)/(math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))))
      
        controlled_omega = omega_ref_old + ( ((c2*vref)*(ye*math.cos(theta_e/2) - xe*math.sin(theta_e/2) ))/( math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))) ) + c3*math.sin(theta_e/2)
        print vref, control_velocity, vact, omega_ref_old, controlled_omega, wact , time_curr
        #*************************************************************************************************
        speed_car.linear.x = control_velocity
        speed_car.angular.z = controlled_omega
        pub_speed.publish(speed_car)

        y_refold = y_ref
        x_refold = x_ref      
        theta_old = theta_up
        time_init = time_curr
        t.sleep()