#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <ros/console.h>
#include <webots_ros/get_float.h>
#include <webots_ros/Float64Stamped.h>

#define TIME_STEP 32
static int controllerCount;
static std::vector<std::string> controllerList;  
static std::string controllerName;
static double GPSValues[3] = {0, 0, 0};
double control_velocity,linear_velocity;
double steering_angle;
// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr &name) {
  controllerCount++;
  controllerList.push_back(name->data);
  ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}

// enable gps
ros::ServiceClient set_GPS_client;
webots_ros::set_int GPS_srv;
ros::Subscriber sub_GPS;

// setting speed
ros::ServiceClient setspeedClient;
webots_ros::set_float setspeed;

//setting steering angle
ros::ServiceClient setsteerClient;
webots_ros::set_float setsteer;

// time step 
ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;

//simulation  time
ros::ServiceClient get_timeClient;
webots_ros::get_float get_timeSrv;

//callback for GPS values from topic 
void GPSCallback(const sensor_msgs::NavSatFix::ConstPtr &values) {
  GPSValues[0] = values->latitude;
  GPSValues[1] = values->altitude;
  GPSValues[2] = values->longitude;
} 

void linear(const webots_ros::Float64Stamped::ConstPtr &values){
   linear_velocity = values->data;
}

//callback for getting linear velocity and angular velocity of the car from python node 
void car_callback(const geometry_msgs::Twist& value)
{
   control_velocity = value.linear.x;
   steering_angle = value.angular.z;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv,"pub", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Rate loop_rate(2);
    ros::Subscriber nameSub = n.subscribe("model_name", 100, controllerNameCallback);
    ros::Subscriber car_speed= n.subscribe("car_speed",100,car_callback);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",100);
    while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
          ros::spinOnce();
          ros::spinOnce(); 
          ros::spinOnce();
  }
    ros::spinOnce();

  // if there is more than one controller available, let the user choose
  if (controllerCount == 1)
    controllerName = controllerList[0];
  else {
    int wantedController = 0;
    std::cout << "Choose the # of the controller you want to use:\n";
    std::cin >> wantedController;
    if (1 <= wantedController && wantedController <= controllerCount)
      controllerName = controllerList[wantedController - 1];
    else {
      ROS_ERROR("Invalid number for controller choice.");
      return 1;
    }
  }

   // leave topic once it's not necessary anymore
  nameSub.shutdown();
  timeStepClient = n.serviceClient<webots_ros::set_int>(controllerName + "/robot/time_step");
  set_GPS_client = n.serviceClient<webots_ros::set_int>(controllerName + "/gps/enable");
  setspeedClient = n.serviceClient<webots_ros::set_float>(controllerName + "/automobile/set_cruising_speed");
  setsteerClient = n.serviceClient<webots_ros::set_float>(controllerName +"/automobile/set_steering_angle");
  get_timeClient = n.serviceClient<webots_ros::get_float>(controllerName + "/robot/get_time");
  ros::Subscriber linearVel = n.subscribe(controllerName + "/automobile/current_speed",10,linear);
  


  GPS_srv.request.value = TIME_STEP;
  set_GPS_client.call(GPS_srv);
 
  timeStepSrv.request.value = TIME_STEP;
  timeStepClient.call(timeStepSrv);
  
  sub_GPS = n.subscribe(controllerName + "/gps/values", 1, GPSCallback);
  ros::Time current_time;
  current_time = ros::Time::now();

  while(1){
    get_timeClient.call(get_timeSrv);
    double simulation_time = get_timeSrv.response.value;
    ROS_INFO("time is %f.",simulation_time);
     
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = GPSValues[0];
    odom.pose.pose.position.y = GPSValues[2];
    odom.pose.pose.position.z = simulation_time;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    // odom.twist.twist.angular.x = 0.0;
    odom_pub.publish(odom);
    ros::spinOnce();
    
    setspeed.request.value = control_velocity;
    setsteer.request.value = steering_angle;
    
    setspeedClient.call(setspeed);
    setsteerClient.call(setsteer);
  }
}