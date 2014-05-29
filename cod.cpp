#include "ros/ros.h"
#include "std_msgs/String.h"


void streamOUTCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Message received from anaysis node: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "cod");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/rod/analysis/streamOUT", 1000, streamOUTCallback);
  ros::spin();

  return 0;
}
