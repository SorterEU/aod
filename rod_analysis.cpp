#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

//globals:
ros::Publisher chatter_pub;
void analysis();
void sendCallback();

void callback_streamIN(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
	analysis();
	sendCallback();
}

void sendCallback()
{
	std_msgs::String msg;
	std::stringstream ss;
	ss << "hello world ";
	msg.data = ss.str();

	ROS_INFO("%s", msg.data.c_str());
	chatter_pub.publish(msg);
}

void analysis()
{
	//todo
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "analysis");
  ros::NodeHandle n;

  //advertise
   chatter_pub = n.advertise<std_msgs::String>("analysis/streamOUT", 1000);

  //subscribes
  ros::Subscriber sub = n.subscribe("/rod/analysis/streamIN", 1000, callback_streamIN);





  ros::spin();


  return 0;
}

