//node of the processing of the IK function
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "robotic_arm/ikine_6dof.h"
#include <iostream>

void number_callback(const std_msgs::Int32::ConstPtr& msg)
{
  ROS_INFO("Received [%d]", msg->data) ;
}


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "pro_arm");
  ros::NodeHandle nh ;
  ros::Subscriber sub= nh.subscribe("/numbers",10, number_callback);
  ros::service::waitForService("robotic_arm/ikine_6dof") ;


  ros::Publisher pub= nh.advertise<std_msgs::Int32>("/angles",10);
  ros::Rate loop_rate(10);
  
  while( ros::ok())
  { 
    std_msgs::Int32 msg;
    //IK editing
    
    pub.publish(msg);
    ROS_INFO("WORKING ,%d", msg.data);
    ros::spinOnce();
    loop_rate.sleep();
    }
}

