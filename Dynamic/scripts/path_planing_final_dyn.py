#!/usr/bin/env python

from __future__ import division
import math 
import cmath
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import numpy as np
from numpy import inf
from scipy import *
import time
import matplotlib.pyplot as plt
np.seterr(divide='ignore', invalid='ignore')
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#********************************************* Declrations ***************************************************************************************************************
global xgps, ygps, dyn_x, dyn_y, follow_dis, critical_dis, VMAX, v_obs, x, y
vinit, ainit, VMAX, num_foc, horizon,follow_dis, critical_dis, v_obs = 0, 0, 5.0, 10, 6, 10, 10, 3
pre_state, state, a_i, t0, x_set_init, y_set_init, lane_status = 0, 0, 1, 0, 0, 0, 0
j, l, id1, DISTANCE, Curv_const = 0, 0, 31, 30, 0.025
epsilon = 1e-05
global x_last_2, x_last, x_cur, y_last_2, y_last, y_cur
x_last_2, x_last, x_cur, y_last_2, y_last, y_cur = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
input1 = np.zeros([5,0])
xxxxx, yyyyy , xdu, ydu, TDpp = [], [], [], [], []
i, h, kkkk, m, n = 0, 0, 0, 0, 0 
x = np.zeros([1001,1])
while(m<5001):
    x[h,0]=m
    m=m+5
    h=h+1
y = np.zeros([1001,1])
while(n<5001):
    y[kkkk,0]= 0
    n=n+5
    kkkk=kkkk+1
x_lmost = np.zeros([1001,1])
y_lmost = np.zeros([1001,1])
x_rmost = np.zeros([1001,1])
y_rmost = np.zeros([1001,1])

x_right = np.zeros([1,1001])
y_right = np.zeros([1,1001])
x_left = np.zeros([1,1001])
y_left = np.zeros([1,1001])
a_coff = np.zeros([6,1])
b_coff = np.zeros([6,1])
a_coff_store = np.zeros([6,5])
b_coff_store = np.zeros([6,5])
v_store = np.zeros([num_foc+1,5]) 
x_store = np.zeros([num_foc+1,5])
y_store = np.zeros([num_foc+1,5])
t_store = np.zeros([num_foc+1,5])
cost_store = np.zeros([5,1])
cost_comp = np.zeros([5,1])
for x_c in range(5):
    cost_comp[x_c,0] = 10000000
xd_n = np.zeros([num_foc+1,1])
yd_n = np.zeros([num_foc+1,1])
curvature = np.zeros([num_foc+1,1])         # used in initialization3() function
################## Controller gains###################
c1, c2, c3 = 2, 0.01, 1.1
#*********************************************************************************************************************************************
#***************** This Function deals with calculation curvature values for array of points *************************************************
def mengercurve(xway, yway):
    way = np.zeros([len(xway),1])    
    s=1
    area, s11, s21, s31 = 0.0, 0.0, 0.0, 0.0
    while(s<np.size(xway)-1):
        area = 0.5*(xway[s-1]*(yway[s] - yway[s+1]) + xway[s]*(yway[s+1] - yway[s-1]) + xway[s+1]*(yway[s-1]-yway[s]))
        s11= math.sqrt(pow((xway[s-1]-xway[s]),2)+ pow((yway[s-1] - yway[s]),2))
        s21= math.sqrt(pow((xway[s+1]-xway[s]),2) + pow((yway[s+1] - yway[s]),2))
        s31= math.sqrt(pow((xway[s-1]-xway[s+1]),2) + pow((yway[s-1] - yway[s+1]),2))
        way[s,0] = (4*area)/(s11*s21*s31)
        s = s+1
    way[0,0] = way[1,0]
    way[np.size(xway)-1,0] = way[np.size(xway)-2,0]  
    return way
#************************************* Velocity Profile **************************************************************************************
def velocity(xv, yv, v_init, a_init, t_init, state):
    global dyn_x, dyn_y, follow_dis, critical_dis, VMAX, v_obs
    velv = np.zeros([np.size(xv),1])         # 6x1
    tim = np.zeros([np.size(xv),1])          # 6x1
    a_long = np.zeros([np.size(xv),1])       # 6x1
    w = np.zeros([np.size(xv),1])            # 6x1    
    a_lat_max = 0.2
    a_long_max= 0.5
    velv[0,0] = v_init   # Initial values
    tim[0,0] = t_init
    a_long[0,0] = a_init
    kkk = mengercurve(xv, yv)
    if(state == 0):                                                # No obstacle case
        v_max = VMAX
        acc_long = a_long_max                                      # depending on state the velocity and a_long is publish

    elif(state == 1):                                              # Follow a car in a lane        
        v_max = min(v_obs,VMAX)
        acc = 0.5*((pow(v_max,2) - pow(v_init,2))/(math.sqrt(pow((xv[0] - dyn_x),2) + pow((yv[0]-dyn_y),2)) - follow_dis ))  
        acc_long = min(acc, a_long_max)
        
    elif(state == 2):                                              # overtacking case
        v_max = VMAX
        acc_long = a_long_max
        
    elif(state == 3):                                              # Emergency Break to stop completely due to pedestrian on road
        dis = math.sqrt( pow( (xv[0]-dyn_x),2 ) + pow ( (yv[0]-dyn_y),2 )) - critical_dis
        v_max = 0.0
        acc_long = max( -pow(v_init,2)/(2*abs(dis)) , -VMAX)
        
    elif(state == 5):                                              # Horizon is exceeded
        v_max = VMAX
        acc_long = a_long_max
    w[0,0] = kkk[0,0]*v_init    
    i=1    
    while(i<(np.size(xv))): 
        if(kkk[i,0]!=0):            # This part deals with the calculation on curve road segment
            v_all = min( v_max, math.sqrt( a_lat_max/abs(kkk[i, 0])) )       
            temp = math.sqrt(max( pow(velv[i-1,0],2) + 2*acc_long*max(math.sqrt(pow((xv[i-1,0]-xv[i,0]),2)+pow((yv[i-1,0] - yv[i,0]),2)), 0), 0))
            velv[i,0] = max(min(temp, v_all),0) 
            temp1 = (pow(velv[i,0],2) - pow(velv[i-1,0],2))/(2*math.sqrt(pow((xv[i-1,0] - xv[i,0]), 2) + pow(( yv[i-1,0] - yv[i,0]) ,2)))
            a_long[i,0] = min(temp1, acc_long)            
            w[i,0] = kkk[i,0]*velv[i,0]
            if(velv[i,0] == velv[i-1,0] and velv[i,0]==0):
                tim[i,0]= tim[i-1,0] + 1
            elif(velv[i,0] == velv[i-1] and velv[i]!=0):
                tim[i,0] = tim[i-1,0] + ((math.sqrt(pow((xv[i-1,0] - xv[i,0]),2)  + pow((yv[i-1,0]-yv[i,0]),2) ))/velv[i-1,0])
            else:
                tim[i,0] = (tim[i-1,0] + ((velv[i,0] - velv[i-1,0])/a_long[i,0]))
            
        else:                        # this part deals with the calculation on st. road segment 
            temp = math.sqrt(max( pow(velv[i-1,0],2) + 2*acc_long*max(math.sqrt(pow((xv[i-1,0]-xv[i,0]),2)+pow((yv[i-1,0] - yv[i,0]),2)), 0), 0))
            velv[i,0] = max(min(temp, v_max),0) 
            temp1 = (pow(velv[i,0],2) - pow(velv[i-1,0],2))/(2*math.sqrt(pow((xv[i-1,0] - xv[i,0]), 2) + pow(( yv[i-1,0] - yv[i,0]) ,2)))
            a_long[i,0] = min(temp1, acc_long)
            w[0, 0] = 0
            w[i,0] = 0
            if(velv[i,0] == velv[i-1,0] and velv[i,0]==0):
                tim[i,0]= tim[i-1,0] + 1
            elif(velv[i,0] == velv[i-1] and velv[i]!=0):
                tim[i,0] = tim[i-1,0] + ((math.sqrt(pow((xv[i-1,0] - xv[i,0]),2)  + pow((yv[i-1,0]-yv[i,0]),2) ))/velv[i-1,0])
            else:
                tim[i,0] = (tim[i-1,0] + ((velv[i,0] - velv[i-1,0])/a_long[i,0]))
        i = i+1
    velv[0,0] = v_init
    tim[0,0] = t_init
    a_long[0,0] = a_init
    
    return xv, yv, velv, tim, a_long, w
#**********************************************************************************************************************************************
# For single value calculation
def menger(x,y):
    global x_last_2, x_last, x_cur, y_last_2, y_last, y_cur
    x_last_2, x_last, x_cur, y_last_2, y_last, y_cur = x_last, x_cur, x, y_last, y_cur, y
    area = (0.5 * (x_last_2 * (y_last - y_cur) + x_last * (y_cur - y_last_2) + x_cur * (y_last_2 - y_last)))
    s1 = math.sqrt((x_last - x_last_2) ** 2 + (y_last - y_last_2) ** 2)
    s2 = math.sqrt((x_cur - x_last) ** 2 + (y_cur - y_last) ** 2)
    s3 = math.sqrt((x_cur - x_last_2) ** 2 + (y_cur - y_last_2) ** 2)
    if s1 <= epsilon:
        return 0.
    if s2 <= epsilon:
        return 0
    if s3 <= epsilon:
        return 0.
    else:
        return 4. * area /(s1 * s2 * s3)
#*********************************************************************************************************************************************
# This Function is for state 0 when there is no obstacle on reference line
def Initialization1(vel, a_long, w , t, i, index, theta): 
    global x, y 
    traj = 1
    V_i, V_f, T_i, T_f = vel[i], vel[i+1], 0, round(float(t[i+1] - t[i]), 4) 
    W_i, W_f, A_i, A_f = w[i], w[i+1], a_long[i], a_long[i+1]
    m_tang2 = ( (y[index+2] - y[index+1]) / ( x[index+2] - x[index + 1]) )
    if(x[index +1] - x[index] < 0):
        th_2 = math.pi + math.atan( m_tang2 )
    else:
        th_2 = math.atan( m_tang2 )
    P = np.matrix([[1,T_i,pow(T_i,2),pow(T_i,3),pow(T_i,4),pow(T_i,5)],
                    [0,1,2*T_i, 3*pow(T_i,2),4*pow(T_i,3),5*pow(T_i,4)],
                    [0,0,2,6*T_i,12*pow(T_i,2),20*pow(T_i,3)],
                    [1,T_f,pow(T_f,2), pow(T_f,3), pow(T_f,4),pow(T_f,5)],
                    [0,1,2*T_f,3*pow(T_f,2), 4*pow(T_f,3), 5*pow(T_f,4)],
                    [0,0,2,6*T_f,12*pow(T_f,2),20*pow(T_f,3)]], dtype='float')
    
    vx_i, vx_f = V_i*math.cos(theta), V_f*math.cos(th_2)
    vy_i, vy_f = V_i*math.sin(theta), V_f*math.sin(th_2)
    Ax_i, Ax_f = ( A_i*math.cos(theta) - V_i*math.sin(theta)*W_i), ( A_f*math.cos(th_2) - V_f*math.sin(th_2)*W_f )
    Ay_i, Ay_f = ( A_i*math.sin(theta) + V_i*math.cos(theta)*W_i ), (  A_f*math.sin(th_2) + V_f*math.cos(th_2)*W_f )
    x_set = x[index+1] 
    y_set = y[index+1]

    return vx_i, vx_f, vy_i, vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_i, T_f, V_f, A_f, x_set, y_set, P, traj
#***********************************************************************************************************************************************
# This function gives the initialization for state 2, phase 1 when the obstacle is present on the reference line
def Initialization2(vel, a_long, w, i, dist, itr2, vd_n, wd_n, ad_n, index, theta, theta4, itr4, ll):
    global x, y, dyn_x, dyn_y, VMAX
    l_off, traj = 2, 5
    if(itr4 == 1):
        ll = ll+1
    ee = np.argmin(np.abs(x - dyn_x)) # search for Obstacle point
    if(itr4 == 1):
        cc = ee+ll
        itr4 = 0
    else:
        cc = ee
    if( x[cc] > dyn_x and y[cc] == dyn_y):
        m_tang3 = ( (y[cc] - dyn_y)/(x[cc] - dyn_x) )
    elif(x[cc]<= dyn_x and y[cc] == dyn_y):
        m_tang3 = ( (y[cc+1] - dyn_y)/ ( x[cc+1] - dyn_x ) )
    elif(y[cc]!=dyn_y):
        m_tang3 = ( (y[cc+1] - y[cc])/(x[cc+1] - x[cc]) )
    m_perp3 = -1/m_tang3
    if( (x[index+1] - x[index]) < 0):
        th_3 = math.pi + math.atan(m_tang3)
    else:
        th_3 = math.atan(m_tang3)
    itr2 = itr2 + 1 
    if(x[cc]< dyn_x and y[cc] == dyn_y):
        xf = [ float(x[cc]), float(dyn_x), float(x[cc+1]) ]
        yf = [ float(y[cc]), float(dyn_y), float(y[cc+1]) ]
        x_f = x[cc+1]
        y_f = y[cc+1]
    elif(x[cc]>dyn_x and y[cc] == dyn_y):
        xf = [ float(x[cc-1]), float(dyn_x), float(x[cc]) ]
        yf = [ float(y[cc-1]), float(dyn_y), float(y[cc]) ]
        x_f = x[cc]
        y_f = y[cc]
    elif(x[cc] == dyn_x and y[cc] == dyn_y):
        xf = [ float(x[cc-1]), float(dyn_x), float(x[cc+1]) ]
        yf = [ float(y[cc-1]), float(dyn_y), float(y[cc+1]) ]
        x_f = x[cc+1]
        y_f = y[cc+1]
    elif(y[cc] != dyn_y):
        xf = [ float(x[cc-1]), float(x[cc]), float(x[cc+1]) ]
        yf = [ float(y[cc-1]), float(y[cc]), float(y[cc+1]) ]
        x_f = x[cc+1]
        y_f = y[cc+1]
    if(itr2>1):
        V_i = vd_n  
    else:
        V_i = vel[i]
    meng = mengercurve(xf, yf)  # output from menger curve function
    V_f = VMAX
    T_i = 0    
    A_i = a_long[i]
    A_f = 0 
    W_i, W_f = w[i], V_f*meng[1,0]
    # ************************************* Time Updation **************************
    if(itr2>1):
        if(vd_n == V_f):
            T_f = dist/vd_n
        else:
            pq = [ad_n, (2*vd_n), -(2*dist)]
            T = np.roots(pq)
            for n in range(np.size(T)):
                if(not any(imag(T.item(n))) == 1):
                    if(T[n]>0):
                        T_f = T[n]
    elif(itr2 == 1):
        if(V_i == V_f):
           T_f = dist/V_i
        else:
            pq = [A_i, (2*V_i), -(2*dist)]
            T = np.roots(pq)
            for n in range(np.size(T)):
                if(not any(imag(T.item(n))) == 1):
                    if(T[n]>0):
                        T_f = T[n]
    # ******************************************************************************
    P = np.matrix([[1,T_i,pow(T_i,2),pow(T_i,3),pow(T_i,4),pow(T_i,5)],
                  [0,1,2*T_i, 3*pow(T_i,2),4*pow(T_i,3),5*pow(T_i,4)],
                  [0,0,2,6*T_i,12*pow(T_i,2),20*pow(T_i,3)],
                  [1,T_f,pow(T_f,2), pow(T_f,3), pow(T_f,4),pow(T_f,5)],
                  [0,1,2*T_f,3*pow(T_f,2), 4*pow(T_f,3), 5*pow(T_f,4)],
                  [0,0,2,6*T_f,12*pow(T_f,2),20*pow(T_f,3)]], dtype='float')
    if(itr2 == 1):
        vx_i, vx_f = V_i*math.cos(theta), V_f*math.cos(th_3)
        vy_i, vy_f = V_i*math.sin(theta), V_f*math.sin(th_3)
        Ax_i, Ax_f = ( A_i*math.cos(theta) - V_i*math.sin(theta)*W_i), ( A_f*math.cos(th_3) - V_f*math.sin(th_3)*W_f )
        Ay_i, Ay_f = ( A_i*math.sin(theta) + V_i*math.cos(theta)*W_i ), (  A_f*math.sin(th_3) + V_f*math.cos(th_3)*W_f ) 
    else:
        vx_i, vx_f = vd_n*math.cos(theta4), V_f*math.cos(th_3)
        vy_i, vy_f = vd_n*math.sin(theta4), V_f*math.sin(th_3)
        Ax_i, Ax_f = ( ad_n*math.cos(theta4) - vd_n*math.sin(theta4)*W_i), ( A_f*math.cos(th_3) - V_f*math.sin(th_3)*W_f )
        Ay_i, Ay_f = ( ad_n*math.sin(theta4) + vd_n*math.cos(theta4)*W_i ), (  A_f*math.sin(th_3) + V_f*math.cos(th_3)*W_f )
    x_set = [x_f - 2*math.sin(th_3)*l_off, x_f - math.sin(th_3)*l_off, x_f, x_f + 0.5*math.sin(th_3)*l_off, x_f + math.sin(th_3)*l_off]
    y_set = [y_f + 2*math.cos(th_3)*l_off, y_f + math.cos(th_3)*l_off, y_f, y_f - 0.5*math.cos(th_3)*l_off, y_f - math.cos(th_3)*l_off]
    return vx_i, vx_f, vy_i, vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_i, T_f, V_f, A_f, x_set, y_set, P.round(2), traj, itr2, m_perp3, itr4, ll
#***************************************************************************************************************************************************
# This function gives the initialization for state 2, phase 2 when the ego vehicle wants to go to reference path again from diverted path
def Initialization3(x_init,y_init,vel,vd_n,wd_n,ad_n,index,theta4):
    global x, y, VMAX
    xd_s, yd_s = [], []
    x_set, y_set = np.zeros([1,1]),  np.zeros([1,1])
    num_foc, traj, interation, T_i, j = 10, 1, 0, 0, 0
    while(interation == 0):
        xf = [float(x[index]), float(x[index+1]), float(x[index+2])]
        yf = [float(y[index]), float(y[index+1]), float(y[index+2])]
        dist = math.sqrt( pow( (x_init - x[index+1,0]), 2) + pow((y_init - y[index+1,0]),2) )
        meng = mengercurve(xf, yf)
        if(j>0):
            mtang4 = (y[index+1,0] - y[index,0])/(x[index+1,0] - x[index,0])
        elif(j==0):
            mtang4 = (y[index+2,0] - y[index+1,0])/(x[index+2,0] - x[index+1,0])

        if(x[index+2] - x[index+1] <0):
            theta_4 = math.pi + math.atan(mtang4)
        else:
            theta_4 = math.atan(mtang4)

        V_f = min(vel)
        if(j>0):
            W_f = V_f*meng[0,0]
        elif(j==0):
            W_f = V_f*meng[1,0]
        A_f = 0
        #********************** Time Calculation *****************************
        if(vd_n == V_f):
           T_f = dist/vd_n
        else:
            pq = [ad_n, (2*vd_n), -(2*dist)]
            T = np.roots(pq)
            for n in range(np.size(T)):
                if(not any(imag(T.item(n))) == 1):
                    if(T[n]>0):
                        T_f = T[n]
        #*********************************************************************
        P = np.matrix([[1,T_i,pow(T_i,2),pow(T_i,3),pow(T_i,4),pow(T_i,5)],
                      [0,1,2*T_i, 3*pow(T_i,2),4*pow(T_i,3),5*pow(T_i,4)],
                      [0,0,2,6*T_i,12*pow(T_i,2),20*pow(T_i,3)],
                      [1,T_f,pow(T_f,2), pow(T_f,3), pow(T_f,4),pow(T_f,5)],
                      [0,1,2*T_f,3*pow(T_f,2), 4*pow(T_f,3), 5*pow(T_f,4)],
                      [0,0,2,6*T_f,12*pow(T_f,2),20*pow(T_f,3)]], dtype='float')        
        Vx_i  = vd_n*math.cos(theta4)
        Vy_i = vd_n*math.sin(theta4)
        Ax_i = ad_n*math.cos(theta4) - vd_n*math.sin(theta4)*wd_n
        Ay_i = ad_n*math.sin(theta4) + vd_n*math.cos(theta4)*wd_n
        Vx_f = V_f*math.cos(theta_4)
        Vy_f = V_f*math.sin(theta_4)
        Ax_f = A_f*math.cos(theta_4) - V_f*math.sin(theta_4)*W_f
        Ay_f = A_f*math.sin(theta_4) + V_f*math.cos(theta_4)*W_f
        xt_f = x[index+1]
        yt_f = y[index+1]
        g1 = np.matrix([ [float(x_init)], [float(Vx_i)], [float(Ax_i)], [float(xt_f)], [float(Vx_f)], [float(Ax_f)] ], dtype='float')
        g2 = np.matrix([ [float(y_init)], [float(Vy_i)], [float(Ay_i)], [float(yt_f)], [float(Vy_f)], [float(Ay_f)] ], dtype='float')
        a_coff = np.matmul(np.linalg.inv(P), g1)
        b_coff = np.matmul(np.linalg.inv(P), g2)
        xd = np.zeros([num_foc+1,1])
        yd = np.zeros([num_foc+1,1])
        td = T_i
        for f in range(num_foc+1):
            xd[f,0] = np.dot(np.matrix([1, round(td,4), round(pow(td,2),4), round(pow(td,3),4), round(pow(td,4),4), round(pow(td,5),4)]), a_coff)
            yd[f,0] = np.dot(np.matrix([1, round(td,4), round(pow(td,2),4), round(pow(td,3),4), round(pow(td,4),4), round(pow(td,5),4)]), b_coff)
            td = round(td,4) + round(((T_f - T_i)/num_foc),4)
        meger_out = mengercurve(xd, yd)
        curvature[:,0] = meger_out[:,0]
        if(max(curvature) <= Curv_const):
            x_set[0,0] = x[index+1,0]
            y_set[0,0] = y[index+1,0]
            interation = interation + 1
        else:
            index = index+1
            j = j+1
    return Vx_i, Vx_f, Vy_i, Vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_i, T_f, V_f, A_f, x_set, y_set, P.round(2), traj, mtang4
#******************************************************************************************************************************************************
# This is the Cost Function which gives the desired path 
def cost(xd, yd, x_cen, y_cen):
    global dyn_x, dyn_y 
    mengercurve_value = np.zeros([num_foc+1,1]) 
    mengercurve_value = mengercurve(xd, yd)
    if(max(mengercurve_value[:,0])<=Curv_const):
        cost1 = 1/max(np.amin(np.sqrt(np.add(np.square(np.subtract(xd,dyn_x)), np.square(np.subtract(yd,dyn_y)) ))) - 2 , 0.01)
        cost2 = np.sum(np.sqrt(np.add(np.square(np.subtract(x_cen,xd)), np.square(np.subtract(y_cen,yd)))))
        total_cost = 2*cost1 + 0.5*cost2
    else:
        total_cost = 10000000
    return total_cost
#*******************************************************************************************************************************************************
# This function works in phase 1 of state 2. It will calulate the reference equation for phase 1 in state 2
def Overtake(a_coff_store, b_coff_store, x_store, y_store, T_i, k, m_tang, m_perp, index):
    global x, y, VMAX
    theta4 = np.zeros([11,1])
    vd_n, ad_n, wd_n, theta_4n = 0.0, 0.0, 0.0, 0.0 
    traj, num_foc, td_n = 5, 10, T_i
    for tr in range(traj):
        iter1 = 0
        while(iter1 == 0 and tr == k):
            if(m_tang == 0):
                p = [a_coff_store[5,tr], a_coff_store[4,tr], a_coff_store[3,tr], a_coff_store[2,tr], a_coff_store[1,tr], a_coff_store[0,tr] - x[index+1]]
                time = np.roots(p)
                for j in range(np.size(time)):
                    if( not any(imag(time.item(j))) == 1):
                        if( real(time[j]) > 0 ):
                            T_fin = round(real(time[j]),4)
            else:
                p = [(b_coff_store[5,tr]-m_perp*a_coff_store[5,tr]), (b_coff_store[4,tr]-m_perp*a_coff_store[4,tr]),(b_coff_store[3,tr]-m_perp*a_coff_store[3,tr]), (b_coff_store[2,tr]-m_perp*a_coff_store[2,tr]),(b_coff_store[1,tr]-m_perp*a_coff_store[1,tr]), (b_coff_store[0,tr]+m_perp*(-a_coff_store[0,tr]) + x[index +1] - y[index+1])]
                time = np.roots(p)
                for j in range(np.size(time)):
                    if( not any(imag(time.item(j))) == 1):
                        if( real(time[j]) > 0 ):
                            T_fin = round(real(time[j]),4)               
            for ii in range(num_foc+1):
                xd_n[ii,0] = np.matmul(np.matrix([[1,round(td_n,4),round(pow(td_n,2),4), round(pow(td_n,3),4), round(pow(td_n,4),4), round(pow(td_n,5),4)]], dtype='float'),a_coff_store[:,tr]) 
                yd_n[ii,0] = np.matmul(np.matrix([[1,round(td_n,4),round(pow(td_n,2),4), round(pow(td_n,3),4), round(pow(td_n,4),4), round(pow(td_n,5),4)]], dtype='float'),b_coff_store[:,tr])              
                td_n = round(td_n,4) + round(((T_fin - T_i)/num_foc),4)
            curv = mengercurve(xd_n, yd_n)              # calling curvature function
            m1 = np.matrix([0,1,2*T_fin, 3*pow(T_fin,2), 4*pow(T_fin,3), 5*pow(T_fin,4)])
            vd_n = max( min( math.sqrt( pow(np.matmul(m1,a_coff_store[:,tr] ), 2) + pow( np.matmul( m1, b_coff_store[:,tr]) , 2) ), VMAX), 0)
            wd_n = vd_n*curv[num_foc,0]
            a1 = np.matrix([0,0,2,6*T_fin, 12*pow(T_fin,2), 20*pow(T_fin,3)])
            ad_n = min( math.sqrt( pow(np.matmul(a1, a_coff_store[:,tr] ), 2) + pow( np.matmul( a1, b_coff_store[:,tr]) , 2) ), 1.2)
            for jj in range(num_foc):
                if((x[index+1] - x[index])<0):
                    theta4[jj,0] = math.pi + math.atan( (yd_n[jj+1,0] - yd_n[jj,0])/( xd_n[jj+1,0] - xd_n[jj,0]) )
                else:
                    theta4[jj,0] = math.atan( (yd_n[jj+1,0] - yd_n[jj,0])/( xd_n[jj+1,0] - xd_n[jj,0]) )                        
            theta_4n = theta4[num_foc-1,0]   # Here I want the 9th value so because of that only "num_foc-1"
            x_store[:,tr] = xd_n[:,0]
            y_store[:,tr] = yd_n[:,0]
            iter1 = iter1 + 1
    return x_store,y_store,vd_n,ad_n,wd_n,theta_4n,index, T_fin
#*****************************************************************************************************************************************************************************************************
# This function works in phase 2 of state 2. It will calulate the reference equation for phase 2 in state 2
def Overtake_1(a_coff_store, b_coff_store, x_store, y_store, T_i, k, m_tang, m_perp, index):
    global x, y, VMAX
    theta4 = np.zeros([11,1])
    vd_n, ad_n, wd_n, theta_4n = 0.0, 0.0, 0.0, 0.0 
    traj, num_foc, td_n = 1, 10, T_i
    for tr in range(traj):
        iter1 = 0
        while(iter1 == 0):
            if(m_tang == 0):
                p = [a_coff_store[5,tr], a_coff_store[4,tr], a_coff_store[3,tr], a_coff_store[2,tr], a_coff_store[1,tr], a_coff_store[0,tr] - x[index+1]]
                time = np.roots(p)
                for j in range(np.size(time)):
                    if( not any(imag(time.item(j))) == 1):
                        if( real(time[j]) > 0 ):
                            T_fin = round(real(time[j]),4)
            else:
                p = [(b_coff_store[5,tr]-m_perp*a_coff_store[5,tr]), (b_coff_store[4,tr]-m_perp*a_coff_store[4,tr]),(b_coff_store[3,tr]-m_perp*a_coff_store[3,tr]), (b_coff_store[2,tr]-m_perp*a_coff_store[2,tr]),(b_coff_store[1,tr]-m_perp*a_coff_store[1,tr]), (b_coff_store[0,tr]+m_perp*(-a_coff_store[0,tr]) + x[index +1] - y[index+1])]
                time = np.roots(p)
                for j in range(np.size(time)):
                    if( not any(imag(time.item(j))) == 1):
                        if( real(time[j]) > 0 ):
                            T_fin = round(real(time[j]),4)            
            for ii in range(num_foc+1):
                xd_n[ii,0] = np.matmul(np.matrix([[1,td_n,pow(td_n,2), pow(td_n,3), pow(td_n,4), pow(td_n,5)]], dtype='float'),a_coff_store[:,tr]) 
                yd_n[ii,0] = np.matmul(np.matrix([[1,td_n,pow(td_n,2), pow(td_n,3), pow(td_n,4), pow(td_n,5)]], dtype='float'),b_coff_store[:,tr])              
                td_n = td_n + (T_fin - T_i)/num_foc
            curv = mengercurve(xd_n, yd_n)               # calling curvature function
            m1 = np.matrix([0,1,2*T_fin, 3*pow(T_fin,2), 4*pow(T_fin,3), 5*pow(T_fin,4)])
            vd_n = max( min( math.sqrt( pow(np.matmul(m1,a_coff_store[:,tr] ), 2) + pow( np.matmul( m1, b_coff_store[:,tr]) , 2) ), VMAX), 0)
            wd_n = vd_n*curv[num_foc,0]
            a1 = np.matrix([0,0,2,6*T_fin, 12*pow(T_fin,2), 20*pow(T_fin,3)])
            ad_n = min( math.sqrt( pow(np.matmul(a1, a_coff_store[:,tr] ), 2) + pow( np.matmul( a1, b_coff_store[:,tr]) , 2) ), 1.2)
            for jj in range(num_foc):
                if((x[index+1] - x[index])<0):
                    theta4[jj,0] = math.pi + math.atan( (yd_n[jj+1,0] - yd_n[jj,0])/( xd_n[jj+1,0] - xd_n[jj,0]) )
                else:
                    theta4[jj,0] = math.atan( (yd_n[jj+1,0] - yd_n[jj,0])/( xd_n[jj+1,0] - xd_n[jj,0]) )                        
            theta_4n = theta4[num_foc-1,0]               # Here I want the 9th value so because of that only "num_foc-1"
            x_store[:,tr] = xd_n[:,0]
            y_store[:,tr] = yd_n[:,0]
            iter1 = iter1 + 1
    return x_store,y_store,vd_n,ad_n,wd_n,theta_4n,index, T_fin   
#*************************************************************************************************************************************************************************
#Alternate Curvature Function
def curve(x,y):
    kk = np.zeros([np.size(x),1])
    dx = np.gradient(x)
    ddx = np.gradient(dx)
    dy = np.gradient(y)
    ddy = np.gradient(dy)
    for i in range(np.size(x)):
        kk[i] = (dx[i]*ddy[i] - dy[i]*ddy[i])/pow((pow(dx[i], 2) + pow(dy[i], 2)),1.5)
    return kk
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
#*********************************************** Dynamic Obstacle Info Callback Function *********************************************************************************************
def dyn_o(data):
    global dy_x, dy_y
    dy_x = abs(data.data[2])
    dy_y = abs(data.data[1])
#************************************************************** Main Code ************************************************************************************************

if __name__ == "__main__":
    global L, yaw, xact , yact, vact, wact, dyn_x, dyn_y, dy_x, dy_y
    yaw, xact, yact, vact, wact = 0.0, 0.0, 0.0, 0.0, 0.0
    x_set_init, y_set_init = 0, 0
    v_init, t_init, A_init = 0, 0, 1
    x_ref_old, y_ref_old = 0.0, 0.0
    id, id2, L, lane_status, horizon, index, state, pre_state, phase = 23, 41, 2.995, 0, 6, 0, 0, 0, 0
    critical_dis, follow_dis, traj, input1[0], input1[1] = 10, 10, 1, 0, 0
    itr2, itr1, vd_n, wd_n, ad_n, th_4, a = 0, 0, 0, 0, 0, 0, 0
    itr4, ll, tsub = 0, 0, 0    
    xk = [0, 0, (math.pi/6)]  
    rospy.init_node('path_planing_final',anonymous=True)
    pub_graph_data = rospy.Publisher('/graph_data', Float64MultiArray, queue_size= 100)
    rospy.Subscriber('/agent_0/odom', Odometry, datagps)
    pub_speed = rospy.Publisher('/agent_0/cmd_vel',Twist,queue_size=10)
    pub_graph_data_traj = rospy.Publisher('/graph_data_traj', Float64MultiArray, queue_size= 100)
    speed_car = Twist()
    graph_data = Float64MultiArray()
    traj_data = Float64MultiArray()      # New publisher for trajectories
    rospy.Subscriber('/dyn_obs', Float64MultiArray, dyn_o)
    rospy.sleep(2.0)
    while( index < np.size(x)):
        if(state == 5):
            state = pre_state      
        xr, yr, vel, t, a_long, w= velocity(x[index:index+horizon], y[index:index+horizon], v_init, ainit, t_init, state)
        i, tempor, pre_state, itr = 0, pre_state, state, 0
        if(state == 2):              # to initialize the time in state 2 this is the temp variable
                ti = t[i]
        while(state == pre_state):
            dyn_x, dyn_y = dy_x, dy_y
            m_tang = ( y[index+1, 0] - y[index] )/ ( x[index + 1] - x[index] ) 
            m_perp = -1/m_tang
            xNormalLine = (1/m_perp)*( y - y[index+1])
            if((x[index+1] - x[index]) < 0):
                theta = math.pi + math.atan(m_tang)
            else:
                theta = math.atan(m_tang)
            if( state != 2 and phase == 0 ):
                vx_i, vx_f, vy_i, vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_i, T_f, V_f, A_f, x_set, y_set, P, traj = Initialization1( vel, a_long, w, t, i, index, theta) 
            elif( state == 2 and phase == 2):
                vx_i, vx_f, vy_i, vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_i, T_f, V_f, A_f, x_set, y_set, P, traj, mtang4 = Initialization3(x_set_init, y_set_init, vel, vd_n, wd_n, ad_n, index, th_4) # call for Initialization3 function
            elif( state == 2 and phase == 1):
                vx_i, vx_f, vy_i, vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_i, T_f, V_f, A_f, x_set, y_set, P, traj, itr2, m_perp3, itr4, ll = Initialization2( vel, a_long, w, i, dist, itr2, vd_n, wd_n, ad_n, index, theta, th_4, itr4, ll)
                xNormalLine1 = (1/m_perp3)*(y - dyn_y) + dyn_x
            for h in range(traj):
                xt_f = x_set[h]
                yt_f = y_set[h]
                if(state == 2 and phase == 2): 
                    g1 = np.matrix([ [round(float(x_set_init),4)], [round(float(vx_i),4)], [round(float(Ax_i),4)], [round(float(xt_f),4)], [round(float(vx_f),4)], [round(float(Ax_f),4)] ], dtype='float')
                    g2 = np.matrix([ [round(float(y_set_init),4)], [round(float(vy_i),4)], [round(float(Ay_i),4)], [round(float(yt_f),4)], [round(float(vy_f),4)], [round(float(Ay_f),4)] ], dtype='float')                    
                    a_coff = np.matmul(np.linalg.inv(P),g1)
                    b_coff = np.matmul(np.linalg.inv(P),g2)
                else:
                    g1 = np.matrix([ [round(float(x_set_init),4)], [round(float(vx_i),4)], [round(float(Ax_i),4)], [round(float(xt_f),4)], [round(float(vx_f),4)], [round(float(Ax_f),4)] ], dtype='float')
                    g2 = np.matrix([ [round(float(y_set_init),4)], [round(float(vy_i),4)], [round(float(Ay_i),4)], [round(float(yt_f),4)], [round(float(vy_f),4)], [round(float(Ay_f),4)] ], dtype='float')
                    a_coff = np.matmul(np.linalg.inv(P), g1)
                    b_coff = np.matmul(np.linalg.inv(P), g2)
                xd, yd = np.zeros([num_foc+1,1]), np.zeros([num_foc+1,1])
                xd_o, yd_o = np.zeros([num_foc+1,1]), np.zeros([num_foc+1,1])
                x_cen, y_cen = np.zeros([num_foc+1,1]),np.zeros([num_foc+1,1])
                TD, vd, wd = np.zeros([num_foc+1,1]), np.zeros([num_foc+1,1]), np.zeros([num_foc+1,1])
                td, xpoint, ypoint, xpnt, ypnt = T_i, np.zeros([num_foc+1,1]), np.zeros([num_foc+1,1]), np.zeros([num_foc+1,1]), np.zeros([num_foc+1,1])

                for f in range(num_foc+1):
                    xd[f,0] = np.dot(np.matrix([1,td,pow(td,2), pow(td,3), pow(td,4), pow(td,5)], dtype='float'), a_coff)
                    yd[f,0] = np.dot(np.matrix([1,td,pow(td,2), pow(td,3), pow(td,4), pow(td,5)], dtype='float'), b_coff)
                    td_old = td
                    if(state == 2 and phase == 2):
                        td = td + (T_f - T_i)/num_foc
                    else:
                        td = td + (T_f - T_i)/num_foc
                    TD[f,0] = td
                    x_cen[f,0] = x[index,0] + f*((x[index + 1] - x[index,0])/num_foc)
                    y_cen[f,0] = y[index,0] + f*((y[index + 1] - y[index,0])/num_foc)
                    xd1_o, yd1_o = dyn_x, dyn_y
                if(traj == 1):
                    x_store[:,0], y_store[:,0] = xd[:,0], yd[:,0]
                    for f in range(6):
                        a_coff_store[f,0] = np.asarray(a_coff[f,0])
                        b_coff_store[f,0] = np.asarray(b_coff[f,0])
                    t_store[:,0] = TD[:,0]
                elif(traj == 5):
                    x_store[:,h], y_store[:,h] = xd[:,0], yd[:,0]
                    for f in range(6):
                        a_coff_store[f,h] = np.asarray(a_coff[f,0])
                        b_coff_store[f,h] = np.asarray(b_coff[f,0])
                    t_store[:,h] = TD[:,0]

                d = np.argmin(np.abs(x - dyn_x))       # search for index
                if(m_tang == inf):
                    if( y[d,0] < dyn_y):
                        j = d+1
                    elif( y[d,0]>=dyn_y):
                        j = d
                else:
                    if( x[d,0] < dyn_x):
                        j = d+1
                    elif( x[d,0]>=dyn_x):
                        j = d
                for q in range(num_foc+1):
                    if(index < j):
                        if( ( math.sqrt( pow( (xd[q] - dyn_x), 2) +  pow( (yd[q] - dyn_y), 2) )) < DISTANCE):
                            if( lane_status == 0 ):
                                phase = 1
                                state = 2
                                xpnt[q,0] = xd[q,0]
                                ypnt[q,0] = yd[q,0]
                                for ff in range(11):
                                    if(xpnt[ff,0]>0):
                                        s = ff
                                itr2 = itr2 + 1
                                if(itr2 == 1):
                                    x_store[0:s, h] = xd[0:s, 0]                         
                                    y_store[0:s, h] = yd[0:s, 0]
                                    x_store[s+1:num_foc+1, h] = xd[s,0]
                                    y_store[s+1:num_foc+1, h] = yd[s,0]
                                    t_f = t[i]+t_store[s,0]
                                    break
                                elif(itr2>1):
                                    total_cost = cost(xd, yd, x_cen, y_cen)
                                    break
                            elif( lane_status == 1):
                                state = 1
                        else:
                            phase = 0
                            state = 0 

                    else:
                        if(phase == 2):
                            state = 2
                            break
                        elif( phase == 0):
                            state = 0
                if(itr2 > 1 and phase == 1):
                    cost_store[h,0] = total_cost            
            if( itr2 > 1 and phase == 1):
                if(np.all(cost_store == cost_comp) == True):    # comparing all the elements of the vector
                    itr4 = 1
                    break
                else:
                    k = np.argmin(cost_store)
                    jj = 0
                x_store, y_store, vd_n, ad_n, wd_n, th_4, index, T_add = Overtake(a_coff_store, b_coff_store, x_store, y_store, T_i, k, m_tang, m_perp, index)  # call for Overtake Function                    
            elif(phase == 2):
                x_store, y_store, vd_n, ad_n, wd_n, th_4, index, T_add = Overtake_1(a_coff_store, b_coff_store, x_store, y_store, T_i, k, m_tang, m_perp, index)
            if(state == 2 and itr2>1):
                if(phase == 1):
                    x_set_init = x_store[num_foc,k]
                    y_set_init = y_store[num_foc,k]
                    dist = math.sqrt( pow((x_set_init - dyn_x), 2) + pow((y_set_init - dyn_y),2) )
                elif(phase == 2):
                    x_set_init = x_store[num_foc, 0]
                    
                    y_set_init = y_store[num_foc, 0]
                    dist = math.sqrt( pow( (x_set_init - dyn_x),2) + pow((y_set_init - dyn_y),2))
                    itr1 = itr1 + 1                    
            else:
                x_set_init = x_store[num_foc, 0]
                y_set_init = y_store[num_foc, 0]                
                dist = math.sqrt( pow( (x_set_init - dyn_x), 2) + pow((y_set_init - dyn_y),2) )
        ######################### Print Fianl Output (Trajectories Print) ##########################################################################################################
            for pp in range(traj):
                for qq in range(11):
                    traj_data.data = [x_store[qq,pp], y_store[qq,pp]]
                    pub_graph_data_traj.publish(traj_data)
        ############ Printing Data and sending it to webots also ###################################################################################################################
            if(state == 2 and itr2>1):
                if(phase==1):
                    # print (a_coff_store[:,k])
                    # for p in range(11):
                    #     # print (x_store[p,k])
                    #     graph_data.data = [x_store[p,k], y_store[p,k]]
                    #     pub_graph_data.publish(graph_data)
                    a_coff_real = a_coff_store[:,k]
                    b_coff_real = b_coff_store[:,k]
                    t_i = float(ti)
                    t_f = float(ti)+float(T_add)
                    # print (t_i, t_f, "state 2")
                elif(phase == 2):
                    if(itr1 == 1):
                        # print (a_coff_store[:,k])
                        # print (t[i],t[i+1], T_add, "phase2-1st")
                        # for rr in range(11):
                        #     # print (x_store[rr,k])
                        #     graph_data.data = [x_store[rr,k], y_store[rr,k]]
                        #     pub_graph_data.publish(graph_data)
                        a_coff_real = a_coff_store[:,k]
                        b_coff_real = b_coff_store[:,k]
                        t_i = float(ti)
                        t_f = float(ti)+float(T_add)
                        # print (t_i, t_f, "state 2")
                    elif(itr1 >1):
                        # print (a_coff_store[:,0])
                        # print (t[i],t[i+1],T_add, "phase2-2nd")
                        # for rr in range(11):
                        #     # print (x_store[rr,0])
                        #     graph_data.data = [x_store[rr,0], y_store[rr,0]]
                        #     pub_graph_data.publish(graph_data)
                        a_coff_real = a_coff_store[:,0]
                        b_coff_real = b_coff_store[:,0]
                        t_i = float(ti)
                        t_f = float(ti)+float(T_add)
                        # print (t_i, t_f, "state 2")
                
            else:
                if(itr2 == 1):
                    # print (a_coff_store[:,0])
                    # print (t[i], t_store[s,0], t[i+1], "Detected")
                    # for rr in range(11):
                    #         # print (x_store[rr,0])
                    #         graph_data.data = [x_store[rr,0], y_store[rr,0]]
                    #         pub_graph_data.publish(graph_data)
                    a_coff_real = a_coff_store[:,0]
                    b_coff_real = b_coff_store[:,0]
                    t_i = float(t[i])
                    t_f = float(t[i])+float(t_store[s,0])
                    # print (t_i, t_f, float(t_store[s,0]), "detected Region")
                elif(itr2 == 0):
                    # print (a_coff_store[:,0])
                    # print (t[i], t[i+1], "St. Line")
                    # for rr in range(11):
                    #         # print (x_store[rr,0])
                    #         graph_data.data = [x_store[rr,0], y_store[rr,0]]
                    #         pub_graph_data.publish(graph_data)
                    a_coff_real = a_coff_store[:,0]
                    b_coff_real = b_coff_store[:,0]
                    t_i = float(t[i])
                    t_f = float(t[i+1])
                    # print (t_i, t_f, "state 0")          
            tdd = 0            
            while(tdd < (t_f-t_i)):
                tdd = (rospy.get_time() - t_i)                
                mu1 = np.matrix([0,1,2*tdd, 3*pow(tdd,2), 4*pow(tdd,3), 5*pow(tdd,4)])
                matu1 = np.dot(mu1, a_coff_real)
                matu2 = np.dot(mu1, b_coff_real)
                vref = max(min(math.sqrt( pow(matu1,2) + pow(matu2,2) ),VMAX) , 0)
                xref = np.dot(np.matrix([1, tdd, pow(tdd,2), pow(tdd,3), pow(tdd,4), pow(tdd,5)]), a_coff_real)
                xdu.append(xref)
                yref = np.dot(np.matrix([1, tdd, pow(tdd,2), pow(tdd,3), pow(tdd,4), pow(tdd,5)]), b_coff_real)
                ydu.append(yref)
                TDpp.append(tdd)
                x_ref_new = xref
                y_ref_new = yref
                theta_ref_new = math.atan((y_ref_new - y_ref_old)/(x_ref_new - x_ref_old))
                if theta_ref_new<0:
                    theta_ref_new += 2*math.pi

                theta_act = yaw        # YAW will come from wrapper
                if theta_act< 0:
                    theta_act += 2*math.pi
                
                m_out = menger(xref, yref)
                omega_ref = m_out*vref            # It is right or wrong that we need to verify
                
                xe = float(math.cos(theta_act)*(x_ref_new - xact) + math.sin(theta_act)*(y_ref_new - yact))   # x_act and y_act will come from wrapper code
                ye = float(-(math.sin(theta_act)*(x_ref_new - xact)) + math.cos(theta_act)*(y_ref_new - yact))
                theta_e = float(theta_ref_new - theta_act)
                if theta_e < -math.pi:
                    theta_e+=2*math.pi
                elif theta_e > math.pi:
                    theta_e-=2*math.pi
                control_velocity = (vref) + ((c1*xe)/(math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))))
                controlled_omega = omega_ref + (((c2*vref)*(ye*math.cos(theta_e/2) - xe*math.sin(theta_e/2) ))/( math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))) ) + c3*math.sin(theta_e/2)
                speed_car.linear.x = control_velocity
                speed_car.angular.z = controlled_omega
                pub_speed.publish(speed_car)
                graph_data.data = [xref, yref, xact, yact]  # data for drawing
                print round(float(xe), 4), round(float(ye),4), round(float(theta_e),4), round(float(vref),4), round(float(control_velocity),4), round(float(vact),4), round(float(omega_ref),4), round(float(controlled_omega),4), round(float(wact),4), round(float(rospy.get_time()),4)
                pub_graph_data.publish(graph_data)
                theta_ref_old = theta_ref_new
                x_ref_old = x_ref_new
                y_ref_old = y_ref_new                
                rospy.sleep(0.064)            
# ********************************** State Updation ***************************************************************************************************************            
            if(state == 2 and itr2>1):
                if(phase == 1):
                    if(m_tang == 0):
                        if(x_store[num_foc,k] > dyn_x):
                            phase = 2
                            itr1 = itr1 + 1
                    elif(m_tang == inf):
                        if(y_store[num_foc,k]>dyn_y):
                            phase = 2
                            itr1 = itr1 + 1
                    elif(m_tang != 0):
                        if( (x[index + 1]- x[index])< 0 or ( y[index + 1] - y[index] )< 0 ):
                            if(x_store[num_foc, k] < dyn_x and y_store[num_foc, k] < dyn_y):
                                phase = 2
                                itr1 = itr1 + 1
                        else:
                            if(x_store[num_foc,k] > dyn_x):
                                phase = 2
                                itr1 = itr1 + 1
                elif(phase == 2):                    
                    if(abs(x_store[num_foc, 0] - x[index+1] )< 0.01 and abs(y_store[num_foc,0] - y[index+1])< 0.01 ):  # here we should compare both values but we are not doing so # Problem is here
                        phase = 0
                        state = 0
                        itr = itr + 1          
        
            if(itr2 == 1):              # the time updation when the detection is taking place
                t_init = t_f
            else:
                t_init = t[i+1]
            if(state == 2):
                ti = t_f
            i = i+1
            if(state != pre_state):
                if(phase == 0 and itr != 0):
                    v_init = vd_n
                    A_init = ad_n
                    index = index + 1
                    itr1 = 0
                itr2 = 0
                break
            if (state == 2):             # if horizon exceeds then t_init should be equal to the t_f during the overtaking 
                t_init = t_f
                if(i>=horizon-1):
                    state = 5
            else:
                if(i>=horizon-1):        #  this is in normal state 0 
                    state = 5
            index = index + 1            

            if(phase == 2):
                v_init = vd_n
                A_init = ad_n
            else:
                v_init = V_f
                A_init = A_f
            # rospy.sleep(1)