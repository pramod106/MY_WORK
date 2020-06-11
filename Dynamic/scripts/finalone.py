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

#********************************************* Declrations ***************************************************************************************************************
global xgps
global ygps
global get_time
global current_speed
i=0
h=0
k=0
m=0
x = np.zeros([1001,1])
while(m<5001):
    x[h,0]=m
    m=m+5
    h=h+1
n=0
y = np.zeros([1001,1])
while(n<5001):
    y[k,0]= -2.5
    n=n+5
    k=k+1

x_lmost = np.zeros([1001,1])
y_lmost = np.zeros([1001,1])
x_rmost = np.zeros([1001,1])
y_rmost = np.zeros([1001,1])
i = 1
x_right = np.zeros([1,1001])
y_right = np.zeros([1,1001])
x_left = np.zeros([1,1001])
y_left = np.zeros([1,1001])
VMAX = 8
num_foc = 10
horizon = 6            # It is like Look ahead distance
global dyn_x
dyn_x = 100
global dyn_y
dyn_y = -2.5
v_obs= 1.5
prev_state = 0 
state = 0
a_i= 1
t0 = 0
x_set_init = 0 
y_set_init = -2.5
j=0                     # j=index
l=0                     
id = 31                 # it is for dynamic obstacle position
lane_status = 0         # change it manually for now 
vinit = 0
ainit = 0
w_init = 0
xd = []
xdu = []
yd = []
ydu = []
x_cen = []
x_cenu = []
y_cen = []
y_cenu = []
TDpp = []
Tdu = []
vd = []
vdu = []
a_coff_store = np.zeros([6,5])
b_coff_store = np.zeros([6,5])
cost_store = np.zeros([5,1])
v_store = np.zeros([num_foc+1,5]) 
x_store = np.zeros([num_foc+1,5])
y_store = np.zeros([num_foc+1,5])
t_store = np.zeros([num_foc+1,5])
mengx = np.zeros([3,1])
mengy = np.zeros([3,1])
td = 0
#***************************************** Menger Curve Function for calculating the curvature at way points *****************************************************************
def mengerway(xway,yway):
    way = np.zeros([len(xway),1])
    s=1
    while(s<np.size(xway)-1):
        area = abs(0.5*(xway[s-1,0]*(yway[s,0] - yway[s+1,0]) + xway[s,0]*(yway[s+1,0] - yway[s-1,0]) + xway[s+1,0]*(yway[s-1,0]-yway[s,0])))
        s11= math.sqrt(pow((xway[s-1,0]-xway[s,0]),2)+ pow((yway[s-1,0] - yway[s,0]),2))
        s21= math.sqrt(pow((xway[s+1,0]-xway[s,0]),2) + pow((yway[s+1,0] - yway[s,0]),2))
        s31= math.sqrt(pow((xway[s-1,0]-xway[s+1,0]),2) + pow((yway[s-1,0] - yway[s+1,0]),2))
        way[s,0] = 4*area/(s11*s21*s31)
    way[0,0] = way[1,0]
    way[np.size(xway)-1,0] = way[np.size(xway)-2,0]
    return way 
#********************************************** GPS Callback Function ****************************************************************************************************
def gps(data):
    global xgps
    global ygps
    global get_time
    global current_speed
    gpx=0
    gpy=0
    get_time = 0 
    current_speed = 0
    xgps = data.pose.pose.position.x
    ygps = data.pose.pose.position.y
    get_time = data.pose.pose.position.z
    current_speed = data.twist.twist.linear.x

#********************************************** Velocity Profile **********************************************************************************************************
def velocity(xv,yv,v_init,a_init,t_init,state):
    velv = np.zeros([np.size(xv),1])      
    tim = np.zeros([np.size(xv),1])
    a_long = np.zeros([np.size(xv),1]) 
    w = np.zeros([np.size(xv),0])         
    a_lat_max = 0.7
    a_long_max= 1.2
    velv[0,0] = v_init
    tim[0,0] = t_init
    a_long[0,0] = a_init
    # kkk = mengerway(xv,yv) 
    # w[0,0] = w_init
    if(state == 0):
        v_max = VMAX
        acc_long = a_long_max                                             # depending on state the velocity and a_long is publish
    elif(state == 1):
        v_max = min(v_obs,VMAX)
        acc = 0.5*((pow(v_max,2) - pow(v_init,2))/(math.sqrt(pow((xv[0] - dyn_x),2) + pow((yv[0]-dyn_y),2)) - follow_dis ))  
        acc_long = min(acc, a_long_max)  
    elif(state == 2):
        v_max = VMAX
        acc_long = a_long_max
    elif(state == 3):
        dis = math.sqrt( pow( (xv[0]-dyn1_x),2 ) + pow ( (yv[0]-dyn1_y),2 )) - critical_dis
        v_max = 0.0
        acc_long = max(- pow(v_init,2)/(2*abs(dis)) , -1.2)
    elif(state == 5):
        v_max = VMAX
        acc_long = a_long_max
    i=1
    while(i<(np.size(xv))):        
        temp = math.sqrt(max( pow(velv[i-1,0],2) + 2*acc_long*max(math.sqrt(pow((xv[i-1,0]-xv[i,0]),2)+pow((yv[i-1,0] - yv[i,0]),2)), 0), 0))
        velv[i,0] = max(min(temp,v_max),0) 
        temp1 = (pow(velv[i,0],2) - pow(velv[i-1,0],2))/(2*math.sqrt(pow((xv[i-1,0] - xv[i,0]), 2) + pow(( yv[i-1,0] - yv[i,0]) ,2)))
        a_long[i,0] = min(temp1, acc_long)        
        if(velv[i,0] == velv[i-1,0] and velv[i,0]==0):
            tim[i,0]= tim[i-1,0] + 1
        elif(velv[i,0] == velv[i-1] and velv[i]!=0):
            tim[i,0] = tim[i-1,0] + ((math.sqrt(pow((xv[i-1,0] - xv[i,0]),2)  + pow((yv[i-1,0]-yv[i,0]),2) ))/velv[i-1,0])
        else:
            tim[i,0] = (tim[i-1,0] + ((velv[i,0] - velv[i-1,0])/acc_long))    
        # w[i,o] = kkk[i,0]*velv[i,0]
        i=i+1
    print velv
    return velv,tim,a_long #,w
#****************************************** Menger Curve Function ***********************************************************************************************************
def menger(XM,YM):
    f = 0
    area = abs(0.5*(XM[f,0]*(YM[f+1,0] - YM[f+2,0]) + XM[f+1,0]*(YM[f+2,0] - YM[f,0]) + XM[f+2,0]*(YM[f,0]-YM[f+1,0])))
    s1= math.sqrt(pow((XM[f,0]-XM[f+1,0]),2)+ pow((YM[f,0] - YM[f+1,0]),2))
    s2= math.sqrt(pow((XM[f+2,0]-XM[f+1,0]),2) + pow((YM[f+2,0] - YM[f+1,0]),2))
    s3= math.sqrt(pow((XM[f,0]-XM[f+2,0]),2) + pow((YM[f,0] - YM[f+2,0]),2))
    k= 4*area/(s1*s2*s3)
    return k  
#************************************************* Main Loop *****************************************************************************************************************
if __name__ == "__main__":
    deta_t = 0.032
    global xgps
    global ygps
    xgps = 0
    ygps = 0
    global get_time  
    get_time = 0    # to get the actual time 
    global current_speed
    current_speed = 0.0
    omega_ref = 0.0  # angular speed of vehicle
    theta_ref_old = 0.0  # angular position of vehicle 
    theta_act = 0.0  # Initial value of angular position of vehicle 
    # theta_act = (math.pi/6)
    wheelBase = 2.995   # vehicle parameter  
    tinit = 0   # simulation time passed into velocity planner 
    # td = T_i
    x_act_new = 0.0   # 
    y_act_new = 0.00  #
    y_act = 0.0  #
    x_act = 0.0 #
    y_ref_new = 0.0   # 
    x_ref_new = 0.0   # 
    y_ref_old = 0.0
    # coefficients from control law 
    tu_old = 0.0 
    #************ gain values ************
    #***************1st*******************
    c1 = 6
    c2 = 0.0001
    c3 = 0.01
    #***************2nd********************
    K_x = 5     # K_x : K_theta = 20:1 and K_theta : K_y = 10:1
    K_y = 0.01
    K_theta = 0.1
    #***************3rd********************
    k_x = 20
    k_y = 5
    k_theta = 40
    #***********************************
    x_ref_old = 0.0
    x_act_old = 0.0001
    y_act_old = 0.0001
    theta_act_old = 0.0
    omega_ref_old = 0.0
    vvv = 0
    # td_old = 0.0
    rospy.init_node('updated',anonymous=True) 
    rospy.Subscriber('/odom', Odometry , gps) 
    pub_speed = rospy.Publisher('/car_speed',Twist,queue_size=10)
    speed_car = Twist()
    
    while(j<np.size(x)):
               
        if(state==5):
            state = prev_state
        vel = np.zeros([horizon,1])
        vel , t , acc = velocity(x[j:j+horizon] ,y[j:j+horizon] ,vinit, ainit, tinit, state)
        # kk = menger(x[j:j+horizon],y[j:j+horizon])
        # print kk 
        l = 0 
        tempor= prev_state
        prev_state = state
        while(state == prev_state):
            v_i = vel[l,0]
            v_f = vel[l+1,0]
            T_i= t[l,0]
            T_f = t[l+1,0]
            a_i = acc[l,0]
            a_f = acc[l+1,0]  
            # ww_i = w[l,0]
            # ww_f = w[l+1,0]
            # print T_i
            P = np.matrix([[1,T_i,pow(T_i,2),pow(T_i,3),pow(T_i,4),pow(T_i,5)],
                         [0,1,2*T_i, 3*pow(T_i,2),4*pow(T_i,3),5*pow(T_i,4)],
                         [0,0,2,6*T_i,12*pow(T_i,2),20*pow(T_i,3)],
                         [1,T_f,pow(T_f,2), pow(T_f,3), pow(T_f,4),pow(T_f,5)],
                         [0,1,2*T_f,3*pow(T_f,2), 4*pow(T_f,3), 5*pow(T_f,4)],
                         [0,0,2,6*T_f,12*pow(T_f,2),20*pow(T_f,3)]])

            l_off = 1.2
            cx=0
            cy = 0
            cz = 0
            cv =0
            xp=0
            yp=0
            zp=0
            vp=0
            while(xp<5001):
                x_right[0,cx]=xp
                xp=xp+5
                cx = cx+1
            while(yp<5001):                # print get_time
                # print td 
                y_right[0,cy]=-7.5
                yp=yp+5
                cy=cy+1
            while(zp<5001):
                x_left[0,cz]=zp
                zp=zp+5
                cz=cz+1
            while(vp<5001):
                y_left[0,cv]=0
                vp=vp+5
                cv=cv+1

            m_tang = (y[j+1] - y[j])/(x[j+1] - y[j])
            m_tamg = (y[j+2] - y[j+1])/(x[j+2]- y[j+1])
            # m_perp = -1/m_tang 
            theta = math.atan(m_tang)
            theta2 = math.atan(m_tamg)
            
            vx_i = v_i*math.cos(theta)
            vx_f = v_f*math.cos(theta2)
            vy_i = v_i*math.sin(theta)
            vy_f = v_f*math.sin(theta2)
            ax_i = a_i*math.cos(theta) #- v_i*ww_i*math.sin(theta)
            ax_f =a_f*math.cos(theta2) #- v_f*ww_f*math.sin(theta2)
            ay_i = a_i*math.sin(theta) #- v_i*ww_i*math.cos(theta)
            ay_f = a_f*math.sin(theta2) #- v_f*ww_f*math.cos(theta2) 

            x_lmost[j+1] = x[j+1] - ((x[j+1] - x_left[0,j+1])/l_off)
            x_rmost[j+1] = x[j+1] + ((x_right[0,j+1] - x[j+1])/l_off)
        
            y_lmost[j+1] = y[j+1] - ((y[j+1] - y_left[0,j+1])/l_off)
            y_rmost[j+1] = y[j+1] + ((y_right[0,j+1] - y[j+1])/l_off)
   
            if((x[j+1] - x[j+1]<0)):
                theta = math.pi + math.atan(m_tang)
            else:
                theta = math.atan(m_tang)
        
            x_set = np.matrix([ [ x_lmost.item(j+1) ], [ x_lmost.item(j+1) + 0.8*math.sin(theta2)*l_off ], [ x.item(j+1) ], [ x.item(j+1) + 2.5*math.sin(theta2)*l_off ], [ x_rmost.item(j+1) ] ])
            y_set = np.matrix([ [ y_lmost.item(j+1) ], [ y_lmost.item(j+1) - 0.8*math.cos(theta2)*l_off ], [ y.item(j+1) ], [ y.item(j+1) - 2.5*math.cos(theta2)*l_off ], [ y_rmost.item(j+1) ] ])
            # print y_set
            for h in range(5):
                xt_f = x_set[h,0]
                yt_f = y_set[h,0]
            
                g1 = np.matrix([[x_set_init], [vx_i], [ax_i], [xt_f], [vx_f], [ax_f] ])

                g2 = np.matrix([[y_set_init], [vy_i],[ay_i], [yt_f], [vy_f], [ay_f] ])
                # if(state==2):
                #     print h
                #     print g1
                #     print g2
                    # print T_f - T_i
                    # print v_f , v_i
                print np.linalg.pinv(P)
                a_coff = np.linalg.pinv(P)*g1
                a_coff = np.transpose(a_coff)
                a_coff_store[:,h] = a_coff
                b_coff = np.linalg.pinv(P)*g2
                b_coff = np.transpose(b_coff)
                b_coff_store[:,h] = b_coff

                xd = np.zeros([num_foc+1,1])
                yd = np.zeros([num_foc+1,1])
                xd_o = np.zeros([num_foc+1,1])
                yd_o = np.zeros([num_foc+1,1])
                xd_v = np.zeros([num_foc+1,1])
                yd_v = np.zeros([num_foc+1,1])
                x_cen = np.zeros([num_foc+1,1])
                y_cen = np.zeros([num_foc+1,1])
                TD = np.zeros([num_foc+1,1])
                vd = np.zeros([num_foc+1,1])
                a_coff =  np.transpose(a_coff)
                b_coff =  np.transpose(b_coff)
                td = T_i              
                for f in range(num_foc+1):
                    m1 = np.matrix([0,1,2*td, 3*pow(td,2), 4*pow(td,3), 5*pow(td,4)]) 
                    mat1 = np.matmul(m1,a_coff)
                    mat2 = np.matmul(m1,b_coff)
                    temp5 = max(min(math.sqrt( pow(mat1,2) + pow(mat2,2) ),VMAX) , 0 )
                    xd[f,0] = np.matmul(np.matrix([1,td,pow(td,2), pow(td,3), pow(td,4), pow(td,5)]), a_coff)
                    yd[f,0]= np.matmul(np.matrix([[1,td,pow(td,2), pow(td,3), pow(td,4), pow(td,5)]]),b_coff)
                    if((v_i-v_f < 0) and (f>0)):
                        temp5 = max(vd[f-1],temp5)
                    elif((v_i - v_f >=0) and f>0 ):
                        temp5 = min(vd[f-1],temp5)
                    vd[f] = temp5
                    td = td + ((T_f - T_i)/num_foc)
                    TD[f,0] = td
                    x_cen[f] = x[j] + f*((x[j+1] - x[j])/num_foc)
                    y_cen[f] = y[j] + f*((y[j+1] - y[j])/num_foc) 
                if(math.sqrt( pow((x[j] - dyn_x),2)+pow((y[j] - dyn_y),2))<10):
                    cost1 = 1/max(np.amin(np.sqrt(np.add(np.subtract(xd,dyn_x), np.subtract(yd,dyn_y) )))-2 , 0.01)
                    state = 2
                else:
                    cost1 = 0
                    state = 0
                cost2 = np.sum(np.sqrt(np.add(np.square(np.subtract(x_cen,xd)), np.square(np.subtract(y_cen,yd)))))
                total_cost = 5*cost1 + 0.5*cost2
                cost_store[h] = total_cost
                #storing data
                cost_store[h] = total_cost
                x_store[:,h]= xd[:,0]
                y_store[:,h] = yd[:,0] 
                v_store[:,h] = vd[:,0]
                t_store[:,h] = TD[:,0]
            L = np.argmin(cost_store)
            r = 0
            tu = T_i
            # print  x_store[:,L] , y_store[:,L]
            while(tu<=T_f):
                global get_time
                global current_speed
                tu = get_time
                mu1 = np.matrix([0,1,2*tu, 3*pow(tu,2), 4*pow(tu,3), 5*pow(tu,4)])
                # print a_coff_store[:,L]
                matu1 = np.dot(mu1, a_coff_store[:,L])
                matu2 = np.dot(mu1, b_coff_store[:,L])
                tempu5 = max(min(math.sqrt( pow(matu1,2) + pow(matu2,2) ),VMAX) , 0 )
                xjju = np.dot(np.matrix([1, tu, pow(tu,2), pow(tu,3), pow(tu,4), pow(tu,5)]), a_coff_store[:,L])
                xdu.append(xjju)
                yjju = np.dot(np.matrix([1, tu, pow(tu,2), pow(tu,3), pow(tu,4), pow(tu,5)]), b_coff_store[:,L])
                ydu.append(yjju)
                TDpp.append(tu)
                # if(v_i-v_f < 0):
                #     tempu5 = max(vdu[r-1],tempu5)
                # elif(v_i - v_f >=0):
                #     tempu5 = min(vdu[r-1],tempu5)
                vdu.append(tempu5)
                x_ref_new = xjju
                y_ref_new = yjju
                theta_ref_new = math.atan((y_ref_new - y_ref_old)/(x_ref_new - x_ref_old)) 
                g=0
                mengx[g+2,0] = mengx[g+1,0]
                mengx[g+1,0] = mengx[g,0]
                mengy[g+2,0] = mengy[g+1,0]
                mengy[g+1,0] = mengy[g,0]                
                mengx[g,0] = xjju
                mengy[g,0] = yjju
                kk = menger(mengx,mengy)
                #*************************************ERROR Calculation*****************************************
                theta_e = theta_ref_new - theta_act
                # print theta_ref_new, theta_act , theta_e 
                # xe = float(x_ref_new - x_act)
                # print theta_ref_new
                xe = float(math.cos(theta_act)*(x_ref_new - x_act) + math.sin(theta_act)*(y_ref_new - y_act))
                # print xe             
                # ye = float(y_ref_new - y_act)
                ye = float(-(math.sin(theta_act)*(x_ref_new - x_act)) + math.cos(theta_act)*(y_ref_new - y_act))
                #***********************************************************************************************
                # print xjju,yjju 
                # print tempu5
                # control_velocity = tempu5*math.cos(theta_e) + K_x*xe #1st
                control_velocity = (tempu5) + ((c1*xe)/(math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2))))) #2nd
                # print float(tempu5) , float(control_velocity/1.33) , float(tu)
                # print float(xe) , float(ye) ,float(tu)
                # control_velocity = tempu5*math.cos(theta_e) + k_x*xe #3rd
                # controlled_omega = max(min(omega_ref + (tempu5)*(K_y*ye + K_theta*math.sin(theta_e)),0.7157 ),-0.7157) #1st
                # print omega_ref
                # omega_ref = kk*tempu5
                # print omega_ref
                controlled_omega = max( min(omega_ref + ( ((c2*tempu5)*(ye*math.cos(theta_e/2) - xe*math.sin(theta_e/2) ))/( math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))) ) + c3*math.sin(theta_e/2) ,0.7157) , -0.7157) #2nd
                # controlled_omega = max(min(omega_ref + k_theta*theta_e + tempu5*2.7*k_y*(math.sin(theta_e) / theta_e),0.7157 ),-0.7157) #3rd
                 
                delta = max(min(math.atan((wheelBase*controlled_omega)/(control_velocity)),0.261799),-0.261799) # delta = steering angle
                # delta = abs(delta)
                # delta = math.atan((wheelBase*controlled_omega*1.33)/(control_velocity))
                # print str(xjju),str(yjju)
                # print "++++++++++++++++++"
                # print str(xe), str(ye) , str(theta_e) , str(tu)  
                # print str(control_velocity) ,str(tu)# ,str(tu)
                # print tempu5, str(control_velocity), str(xe) , str(ye) , float(xjju) , float(yjju)
                # if(delta>0):
                #     steer = math.atan( math.tan(round(delta,4)) - (1.628/(2*wheelBase)) )
                # elif(delta<0):
                #     steer = math.atan( math.tan(round(delta,4)) + (1.628/(2*wheelBase)) )
                # print "+++++++++++++++++++++++++"
                # print float(xjju) , float(yjju)
                # print "*************************"
                speed_car.linear.x = control_velocity*3.6
                # print round(delta,2) 
                speed_car.angular.z = delta #round(delta,2) # steer
                pub_speed.publish(speed_car)
                deltat = (tu - tu_old)
                global xgps
                global ygps
                x_act = round(xgps,7)
                y_act = round(ygps,7) 
                # print deltat
                # print theta_act, theta_act_old
                if((deltat == 0) and ((theta_act - theta_act_old)==0 )):
                    omega_ref = omega_ref_old
                    theta_act = theta_act_old
                elif( deltat > 0):
                    omega_ref = (theta_ref_new - theta_ref_old)/deltat
                    # print x_act, x_act_old
                    vvv = (x_act - x_act_old)/deltat 
                    # print vvv
                    # omega_ref = kk*tempu5
                    # theta_act = math.atan((y_act - y_act_old)/(x_act - x_act_old))
                # print omega_ref
               
                x_ref_old = x_ref_new                
                y_ref_old = y_ref_new 
                # print float(tempu5) ,float(vvv) , float(tu) 
                theta_act = math.atan((y_act - y_act_old)/(x_act - x_act_old  + 0.000001 ))
                x_act_old = x_act
                y_act_old = y_act
                tu_old = tu 
                theta_ref_old = theta_ref_new
                theta_act_old = theta_act
                omega_ref_old = omega_ref
                r = r+1
                time.sleep(0.08) 
                # print "******************************************************************"
            vdp =  np.asarray(vdu,dtype=float)
            xdp = np.asarray(xdu,dtype=float)
            ydp = np.asarray(ydu,dtype=float)
            TDp = np.asarray(TDpp,dtype=float)

                        
            x_set_init = x_store.item(num_foc,L) #float(xdp[ np.size(xdu)-1])
            # print x_set_init #,x_store.item(num_foc-1,L)            
            y_set_init = y_store.item(num_foc,L)#float(ydp[ np.size(ydu)-1])#
            # print y_set_init#,y_store.item(num_foc-1,L)
            tinit = T_f #float(TDp[ np.size(TDpp)-1])
            # print tinit #,T_f
            vinit = v_f#float(vdp[ np.size(vdu)-1])
            # print vinit, v_f 
            ainit = a_f
            l = l+1
            j = j+1  
            if(l>=(horizon-2)):
                state=5