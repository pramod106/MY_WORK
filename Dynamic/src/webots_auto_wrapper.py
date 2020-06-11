#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from webots_ros.srv import set_int, set_float, get_float
from webots_ros.msg import Float64Stamped
import math
from tf.transformations import quaternion_about_axis

class Vehicle_Info:

    def __init__(self, agent_str):
        self.update_count = 0
        self.wheel_base = 2.94    # for tesla use 2.94 and for bmw 2.995
        self.agent_str = agent_str
        self.x_cur = 0.
        self.y_cur = 0.
        self.theta_cur = 0.
        self.x_last = 0.
        self.y_last = 0.
        self.theta_last = 0.
        self.x_last_2 = 0.
        self.y_last_2 = 0.
        self.time_cur = 0.
        self.time_last = 0.
        self.del_time = 0.
        self.lin_vel = 0.
        self.ang_vel = 0.
        self.epsilon = 1e-05

    def update_time(self, data):
        if data > self.time_cur:
            self.del_time = data - self.time_cur
            self.time_last = self.time_cur
            self.time_cur = data
            return True
        return False

    def update_state(self, x, y):
        self.update_count += 1
        self.x_last_2 = self.x_last
        self.y_last_2 = self.y_last
        self.x_last = self.x_cur
        self.y_last = self.y_cur
        self.x_cur = x
        self.y_cur = y
        if not(abs(self.x_last_2 - self.x_last) <= self.epsilon and abs(self.y_last_2 - self.y_last) <= self.epsilon):
            self.theta_last = self.theta_cur
        if not(abs(self.x_last - self.x_cur) <= self.epsilon and abs(self.y_last - self.y_cur) <= self.epsilon):
             self.theta_cur = math.atan2((self.y_cur - self.y_last), (self.x_cur - self.x_last))
        self.lin_vel = math.sqrt((self.x_cur - self.x_last) ** 2 + (self.y_cur - self.y_last) ** 2)/self.del_time
        temp = self.menger_curve()
        # print temp
        self.ang_vel = self.lin_vel * temp
    
    def menger_curve(self):
        area = (0.5 * (self.x_last_2 * (self.y_last - self.y_cur) + self.x_last * (self.y_cur - self.y_last_2) + self.x_cur * (self.y_last_2 - self.y_last)))
        s1 = math.sqrt((self.x_last - self.x_last_2) ** 2 + (self.y_last - self.y_last_2) ** 2)
        s2 = math.sqrt((self.x_cur - self.x_last) ** 2 + (self.y_cur - self.y_last) ** 2)
        s3 = math.sqrt((self.x_cur - self.x_last_2) ** 2 + (self.y_cur - self.y_last_2) ** 2)

        # print area, s1, s2, s3
        if s1 <= self.epsilon:
            return 0.
        if s2 <= self.epsilon:
            return 0.
        if s3 <= self.epsilon:
            return 0.
        else:
            return 4. * area /(s1 * s2 * s3)

class Webots_Auto_Wrapper:
    
    def __init__(self):
        step = 16                 # NOTE: step <= basic time step in the simulator
        self.controller_count = 0
        self.controllers = {}

        rospy.init_node('webots_auto_wrapper', anonymous = True)
        mn = rospy.Subscriber('model_name', String, self.callback_model_name)
        rospy.sleep(5.0)
        if self.controller_count == 0:
            print 'Please run Webots with the appropriate world file'
        else:
            mn.unregister()
            # Ros Services
            ts = {}
            gp = {}
            self.lin_vel = {}
            self.ang_vel = {}

            #Vehicle Data
            self.vehicles = {}

            #Publishers
            self.odom_pub = {}

            for i in self.controllers.keys():
                self.vehicles[i] = Vehicle_Info(i)
                rospy.Subscriber('{}/cmd_vel'.format(i), Twist, self.callback_vel, i)
                ts[i] = rospy.ServiceProxy('{}/robot/time_step'.format(self.controllers[i]), set_int)
                ts[i](step)
                gp[i] = rospy.ServiceProxy('{}/gps/enable'.format(self.controllers[i]), set_int)
                gp[i](step)
                self.lin_vel[i] = rospy.ServiceProxy('{}/automobile/set_cruising_speed'.format(self.controllers[i]), set_float)
                self.ang_vel[i] = rospy.ServiceProxy('{}/automobile/set_steering_angle'.format(self.controllers[i]), set_float)    
                rospy.Subscriber('{}/gps/values'.format(self.controllers[i]), NavSatFix, self.callback_gps_pos, i)  
                self.odom_pub[i] = rospy.Publisher('{}/odom'.format(i), Odometry, queue_size = 10)

    def callback_vel(self, data, agent_str):
        lin_vel = data.linear.x * 3.6
        ang_vel = data.angular.z
        steer = math.atan2(ang_vel * self.vehicles[agent_str].wheel_base, data.linear.x)
        self.lin_vel[agent_str](lin_vel)
        self.ang_vel[agent_str](steer)

    def callback_model_name(self, data):
        self.controller_count += 1
        temp = data.data.split('_')
        self.controllers['{}_{}'.format(temp[0], temp[1])] = data.data
        
    def callback_gps_pos(self, data, agent_str):
        secs = data.header.stamp.to_sec()
        if self.vehicles[agent_str].update_time(secs):
            x = data.latitude
            y = data.longitude
            self.vehicles[agent_str].update_state(x, y)

            msg_odom = Odometry()
            msg_odom.header.stamp = rospy.Time.now()
            msg_odom.header.seq = self.vehicles[agent_str].update_count
            
            msg_odom.pose.pose.position.x = self.vehicles[agent_str].x_cur
            msg_odom.pose.pose.position.y = self.vehicles[agent_str].y_cur
            q = quaternion_about_axis(self.vehicles[agent_str].theta_cur, (0, 0, 1))
            msg_odom.pose.pose.orientation.x = q[0]
            msg_odom.pose.pose.orientation.y = q[1]
            msg_odom.pose.pose.orientation.z = q[2]
            msg_odom.pose.pose.orientation.w = q[3]
            
            msg_odom.twist.twist.linear.x = self.vehicles[agent_str].lin_vel
            msg_odom.twist.twist.angular.z = self.vehicles[agent_str].ang_vel

            self.odom_pub[agent_str].publish(msg_odom)



if __name__ == "__main__":
    w = Webots_Auto_Wrapper()
    while not rospy.is_shutdown():
        pass