#!/usr/bin/env python
from __future__ import division
import math 
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import numpy as np
import time
from webots_ros.srv import set_int, set_float, get_float
np.seterr(divide='ignore', invalid='ignore')

#***************************************************************************************************************************************************************************
if __name__ == "__main__":
    tu_old = 0
    x_old = 48.0
    y_old = 0.0
    vref = 10.0
    radius = 48 
    omega_ref_old = vref/radius
    rospy.init_node('ref',anonymous=True)  
      
    pub_speed = rospy.Publisher('/agent_0/odom_ref',Pose,queue_size=10)
    speed_car = Pose()
    
    while(True): 
        now = rospy.get_rostime()
        # tu = now.to_time()
        # timec = rospy.ServiceProxy('/agent_0/robot/get_time', get_float)
        tu = time.time()
        print tu.ask
        del_t = tu - tu_old
        # del_t = 0.05
        # print del_t
        if(del_t==0):
            x_new = x_old
            y_new = y_old 
        elif(del_t>0):
            # print x_old, omega_ref_old
            x_new = x_old + del_t*(-y_old*omega_ref_old)
            y_new = y_old + del_t*(x_old*omega_ref_old)
            # print x_old, y_old, x_new , y_new
            theta_ref = math.pi - math.atan2(x_new,y_new)
            speed_car.position.x = x_new
            speed_car.position.y = y_new
            speed_car.position.z = theta_ref
            pub_speed.publish(speed_car)
            x_old = x_new
            y_old = y_new 
            tu_old = tu  
        time.sleep(0.05)
                    