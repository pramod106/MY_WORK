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
K_X, K_Y, K_TH = 0.25, 0.00125 , 0.0125 
c_x, c_y, c_th = None , None, None 

#********************************************** GPS Callback Function ****************************************************************************************************
def datagps(data):
    global yaw, xact , yact, vact, wact
    roll, pitch = 0.0, 0.0
    orintation_q = data.pose.pose.orientation
    orientation_list = [orintation_q.x, orintation_q.y,orintation_q.z,orintation_q.w]
    (roll , pitch, yaw) = euler_from_quaternion(orientation_list)
    if data.pose.pose.position.x and data.pose.pose.position.y != 0:
        xact =  data.pose.pose.position.x
        yact = data.pose.pose.position.y
        vact = data.twist.twist.linear.x
        wact = data.twist.twist.angular.z 
    # print xact, yact
#*************************************************************************************************************************************************************************

if __name__ == "__main__":
    global xact, yact, yaw , vact, wact
    xact, yact , yaw, vact, wact = 0.0, 0.0, 0.0, 0.0, 0.0 
    vref, acc, control_velocity_old = 8, 0.0002, 8
    #************************St. Line **************************************
    xact_st, yact_st = xact, yact
    x_ref_st, y_ref_st = 0.0 , 0.0
    x_refold_st, y_refold_st, theta_old_st = 0.0, 0.0, 0.0
    #*********** Circle ****************************************************
    Radius = 40.0
    xact_cir, yact_cir = xact, yact
    x_ref_cir, y_ref_cir = Radius , 0.0
    x_refold_cir, y_refold_cir, theta_old_cir = Radius, 0.0, 0.0
    theta_up_cir = 0.0 
    #************* Ellipse ************************************************* 
    major_x, minor_y = 60.0, 40.0  
    xact_ell, yact_ell = xact, yact
    x_ref_ell, y_ref_ell = major_x, 0.0
    x_refold_ell, y_refold_ell, theta_old_ell = major_x, 0.0, 0.0
    theta_up_ell = 0.0    
    #************* Sinusoidal **********************************************
    Amplitude, Lob = 5, 60  
    xact_sin, yact_sin = xact, yact
    x_ref_sin, y_ref_sin = 0.0 , 0.0
    x_refold_sin, y_refold_sin, theta_old_sin = 0.0, 0.0, 0.0
    theta_up_sin = 0.0 
    #************* Lissagious **********************************************
    a_x, b_x = 60, 50
    xact_Lis, yact_Lis = xact, yact
    x_ref_Lis, y_ref_Lis = a_x , b_x
    x_refold_Lis, y_refold_Lis, theta_old_Lis = a_x, b_x, 0.0
    theta_up_Lis = 0.0 
    #***********************************************************************
    rospy.init_node('check',anonymous=True) 
    rospy.Subscriber('/agent_0/odom', Odometry, datagps)
    pub_speed = rospy.Publisher('/agent_0/cmd_vel',Twist,queue_size=10)
    pub_graph_data = rospy.Publisher('/graph_data', Float64MultiArray, queue_size= 100)
    graph_data = Float64MultiArray()
    # pub_act = rospy.Publisher('/pub_data', Pose, queue_size=10 )
    # t = rospy.Rate(50) 
    speed_car = Twist()
    # act_data = Pose()
    time.sleep(5.0)
    time_init = rospy.get_time() 

    while True:
        time_curr = rospy.get_time()
        deltat = time_curr - time_init
        global xact, yact, yaw, vact, wact
        #**************************************St. line *****************************************************************   
        # y_ref_st = 0.0
        # x_ref_st = vref*(time_curr - time_init)  
        # omega_ref_old_st = 0.0  
        # theta_ref_new = math.atan2( (y_ref_st - y_refold_st), (x_ref_st - x_refold_st) )
        # if theta_ref_new < 0:
        #     theta_ref_new += 2*math.pi
        #************************************** Circle Curve ************************************************************
        # theta_dot_up_cir = (vref/Radius)                                                                                 
        # theta_up_cir = theta_old_cir + deltat*theta_dot_up_cir   
        # y_ref_cir = Radius*math.sin(theta_up_cir)
        # x_ref_cir = Radius*math.cos(theta_up_cir)  
        # omega_ref_old_cir = (vref/Radius)  
        # theta_ref_new = math.atan2( (y_ref_cir - y_refold_cir), (x_ref_cir - x_refold_cir) )
        # if theta_ref_new < 0:
        #     theta_ref_new += 2*math.pi
        # #*********************************************** Ellipse ********************************************************
        # theta_dot_up_ell = vref/( math.sqrt( pow(major_x ,2) + pow(minor_y,2) - pow(x_ref_ell,2) - pow(y_ref_ell,2) ) )   
        # theta_up_ell = theta_old_ell + deltat*theta_dot_up_ell
        # y_ref_ell = minor_y*math.sin(theta_up_ell)
        # x_ref_ell = major_x*math.cos(theta_up_ell) 
        # omega_ref_old_ell = ( ( (major_x*minor_y)*theta_dot_up_ell )/( pow(major_x,2) + pow(minor_y ,2) - pow(x_ref_ell,2) - pow(y_ref_ell,2) ) )
        # theta_ref_new = math.atan2( (y_ref_ell - y_refold_ell), (x_ref_ell - x_refold_ell) )
        # if theta_ref_new<0:
        #     theta_ref_new += 2*math.pi
        # #*********************************************** Sinusoidal *****************************************************
        # theta_dot_up_sin = vref/( math.sqrt( pow(Amplitude,2)*pow(math.cos(theta_up_sin),2) + ( pow(Lob,2)/pow(math.pi,2) ) )) 
        # theta_up_sin = theta_old_sin + deltat*theta_dot_up_sin
        # y_ref_sin = Amplitude*math.sin(theta_up_sin)
        # x_ref_sin = (Lob*theta_up_sin)/(math.pi) 
        # omega_ref_old_sin = -(Amplitude*Lob*theta_dot_up_sin*math.sin(theta_up_sin)/ (math.pi*( ( pow(Lob,2)/pow(math.pi,2) ) + pow(Amplitude,2)*pow( math.cos(theta_up_sin),2 ) )))       
        # theta_ref_new = math.atan2( (y_ref_sin - y_refold_sin), (x_ref_sin - x_refold_sin) )
        # if theta_ref_new<0:
        #     theta_ref_new += 2*math.pi
        # #*********************************************** Lissagious *****************************************************
        theta_dot_up_lis = vref/( math.sqrt( pow( (a_x*math.sin(theta_up_Lis)) , 2)  + pow( ( 2*b_x*math.cos(2*theta_up_Lis) ), 2 ) ) )                                                                                         
        theta_up_Lis = theta_old_Lis + deltat*theta_dot_up_lis
        y_ref_Lis = b_x*math.sin(2*theta_up_Lis)
        x_ref_Lis = a_x*math.cos(theta_up_Lis)
        omega_ref_old_Lis = ( (2*a_x*b_x*theta_dot_up_lis*( math.cos(theta_up_Lis) + math.sin(theta_up_Lis)*math.sin(2*theta_up_Lis) ))/( pow( (a_x*math.sin(theta_up_Lis)) ,2)+pow( (2*b_x*math.cos(2*theta_up_Lis)), 2 ) ))
        theta_ref_new = math.atan2( (y_ref_Lis - y_refold_Lis), (x_ref_Lis - x_refold_Lis) )
        if theta_ref_new<0:
            theta_ref_new += 2*math.pi
        
        #****************************************************************************************************************
        theta_act = yaw
        if theta_act< 0:
            theta_act += 2*math.pi
        
        omega_ref = omega_ref_old_Lis              # change this when you want to check for different curves
        x_ref = x_ref_Lis
        y_ref = y_ref_Lis    
        # print theta_ref_new, theta_act
        #*************************************************************************************************
        xe = float(math.cos(theta_act)*(x_ref - xact) + math.sin(theta_act)*(y_ref - yact))        
        ye = float(-(math.sin(theta_act)*(x_ref - xact)) + math.cos(theta_act)*(y_ref - yact))
        theta_e = theta_ref_new - theta_act
        #*************************************************************************************************
        if theta_e < -math.pi:
            theta_e+=2*math.pi
        elif theta_e > math.pi:
            theta_e-=2*math.pi 
        
        control_velocity = (vref) + ((c1*xe)/(math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))))   # controller 1
        # control_velocity = vref*math.cos(theta_e) + K_X*xe                                # controller 2
        # control_velocity = vref*math.cos(theta_e) + c_x*xe                                # Ho-Hoon Lee's Control law   
      
        controlled_omega = omega_ref + (((c2*vref)*(ye*math.cos(theta_e/2) - xe*math.sin(theta_e/2) ))/( math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))) ) + c3*math.sin(theta_e/2) #Controller 1
        # controlled_omega = omega_ref + vref*( K_Y*ye + K_TH*math.sin(theta_e) )             # Controller 2
        # controlled_omega = omega_ref + c_y*vref*ye + c_th*math.sin(theta_e)                 # Ho-Hoon Lee's Control law
        
        # if (control_velocity > (control_velocity_old + acc*deltat)):
        #     control_velocity = control_velocity_old + acc*deltat
        # elif (control_velocity < (control_velocity_old - acc*deltat) ):
        #     control_velocity = control_velocity_old - acc*deltat  
        
        graph_data.data = [x_ref, y_ref, xact, yact]   # Data Publish to plot
        pub_graph_data.publish(graph_data)
        
        print x_ref, y_ref, xact, yact, xe, ye, theta_e, vref, control_velocity, vact, omega_ref, controlled_omega, wact, time_curr
        #*************************************************************************************************
        speed_car.linear.x = control_velocity
        speed_car.angular.z = controlled_omega
        pub_speed.publish(speed_car)   
        #********************************** St. line *****************************************************
        # y_refold_st = y_ref_st   
        # x_refold_st = x_ref_st  
        #********************************** Circle *******************************************************
        # y_refold_cir = y_ref_cir
        # x_refold_cir = x_ref_cir      
        # theta_old_cir = theta_up_cir
        #********************************* Ellipse *******************************************************
        # y_refold_ell = y_ref_ell
        # x_refold_ell = x_ref      
        # theta_old_ell = theta_up_ell
        # #******************************* Sinusoidal ******************************************************
        # y_refold_sin = y_ref_sin
        # x_refold_sin = x_ref_sin      
        # theta_old_sin = theta_up_sin
        # #******************************* Lissgious ******************************************************
        y_refold_Lis = y_ref_Lis
        x_refold_Lis = x_ref_Lis      
        theta_old_Lis = theta_up_Lis
        # #*************************************************************************************************
        time_init = time_curr
        control_velocity_old = control_velocity
        rospy.sleep(0.064)