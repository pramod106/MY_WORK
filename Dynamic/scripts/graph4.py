#!/usr/bin/env python
from __future__ import division
import math 
import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
xe, ye, theta_e, tim = [], [], [], []

def gpsdata(data):
    global vcar, vcont, tim
    
    x_e = data.data[8]
    y_e = data.data[9]
    th_e = data.data[10]

    time = rospy.get_time()
    
    xe.append(x_e)
    ye.append(y_e)
    theta_e.append(th_e)

    tim.append(time)
    
def print_data():
    global  vcar, vcont, tim
    title_font = {'fontname':'Arial', 'size':'35', 'color':'black', 'weight':'normal',
    'verticalalignment':'bottom'} # Bottom vertical alignment for more space
    axis_font = {'fontname':'Arial', 'size':'20'}
    plt.subplot(3,1,1)
    plt.clf()
    plt.plot(tim, xe, label = "Error in X", color= "red")
    plt.title("Curvature = 0.05, Detection Distance = 20m , Vel =  5 m/s, ", **title_font)
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    plt.xlabel("Error in X \n (m)",**axis_font)
    plt.ylabel("Time \n (sec)",**axis_font)
    plt.legend()
    plt.grid()
    plt.show()
    
    plt.subplot(3,1,2)
    plt.clf()
    plt.plot(tim, ye, label = "Error in Y", color = "green")
    plt.plot(tim, theta_e, label = "Error in Theta", color = "blue")
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    plt.xlabel("Error in Y \n (m)",**axis_font)
    plt.ylabel("Time \n (sec)",**axis_font)
    plt.legend()
    plt.grid()
    plt.show()
    
    plt.subplot(3,1,2)
    plt.clf()
    plt.plot(tim, theta_e, label = "Error in Theta", color = "blue")
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    plt.xlabel("Error in Theta \n (rad)",**axis_font)
    plt.ylabel("Time \n (sec)",**axis_font)
    plt.legend()
    plt.grid()

    plt.pause(0.02)     

if __name__ == "__main__":  
    plt.figure()
    rospy.init_node('graph',anonymous=True)
    rospy.Subscriber('/graph_data', Float64MultiArray,gpsdata)
    while True:
        # print vr, vc, vcar, wr, wc, wcar, time
        print_data()
        # rospy.sleep(0.064)