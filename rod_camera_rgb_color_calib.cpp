#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "color_calib");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/rod/camera/rgb/image_calibrated_color", 1000);

  ros::Rate loop_rate(1);

  //test
  int count = 0;
  while (ros::ok())
  {
  
    std_msgs::String msg;

    std::stringstream ss;
    ss << "/rod/camera/rgb/image_calibrated_color " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}