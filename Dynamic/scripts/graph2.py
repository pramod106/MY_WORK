#!/usr/bin/env python
from __future__ import division
import math 
import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
vcar, vcont, tim= [], [], []

def gpsdata(data):
    global vcar, vcont, tim
    
    car_vel = data.data[4]
    car_con_vel = data.data[6]
    time = rospy.get_time()
    
    vcar.append(car_vel)
    vcont.append(car_con_vel)
    tim.append(time)
    
def print_data():
    global  vcar, vcont, tim
    
    plt.clf()
    plt.plot(tim, vcar, color= "red")
    plt.plot(tim, vcont, color = "green")

    plt.legend()
    plt.pause(0.02)     

if __name__ == "__main__":  
    plt.figure()
    rospy.init_node('graph',anonymous=True)
    rospy.Subscriber('/graph_data', Float64MultiArray,gpsdata)
    while True:
        # print vr, vc, vcar, wr, wc, wcar, time
        print_data()
        # rospy.sleep(0.064)