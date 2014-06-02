#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>


//globals:
ros::Publisher chatter_pub;

void image_agregation();
void sendCallback();
void callback_color_calib(const std_msgs::String::ConstPtr&);
void callback_rgb_STFP(const std_msgs::String::ConstPtr&);
void callback_image_proc(const std_msgs::String::ConstPtr&);
void callback_ir_STFP(const std_msgs::String::ConstPtr&);

void image_agregation(){}

void callback_color_calib(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("color_calib ");
	//image_agregation();
	sendCallback();
}

void callback_rgb_STFP(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("rgb_STFP: [%s]", msg->data.c_str());
	image_agregation();
	sendCallback();
}

void callback_ir_STFP(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("ir_STFP: [%s]", msg->data.c_str());
	image_agregation();
	sendCallback();
}

void callback_image_proc(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("image_proc: [%s]", msg->data.c_str());
	image_agregation();
	sendCallback();
}

void sendCallback()
{
	std_msgs::String msg;
	std::stringstream ss;
	ss << "image_aggregation ";
	msg.data = ss.str();

	ROS_INFO("%s", msg.data.c_str());
	ROS_INFO("maksym");
	chatter_pub.publish(msg);
	ROS_INFO("maksym2");
}
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "image_aggregation");
  ros::NodeHandle n;
  
  //advertise
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/rod/analysis/streamIN", 1000);

  //subscribes:
  //ros::Subscriber sub_color_calib = n.subscribe("/rod/camera/rgb/image_calibrated_color", 1000, callback_color_calib);
  //ros::Subscriber sub_rgb_STFP = n.subscribe("/TF", 1000, callback_rgb_STFP);
  //ros::Subscriber sub_ir_STFP = n.subscribe("/TF", 1000, callback_ir_STFP);
  //ros::Subscriber sub_image_proc = n.subscribe("/rod/camera/ir/image_rect_mono", 1000, callback_image_proc);
  
  ros::spin();



  return 0;
}
