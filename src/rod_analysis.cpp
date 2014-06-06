#include "ros/ros.h"
#include "std_msgs/String.h"
#include "NodeClass.h"

#include <sstream>

//globals:

void analysis()
{
	//todo
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "analysis");
  ros::NodeHandle n;

  //Create a new NodeClass object
   NodeClass *node_class = new NodeClass();

   // Create a publisher and name the topic.
  node_class->_pub_message = n.advertise<std_msgs::String>("/rod/analysis/streamOUT", 1000);

  // Tell ROS how fast to run this node.
  ros::Rate loop_rate(10);

  //subscribes
  ros::Subscriber sub = n.subscribe("/rod/analysis/streamIN", 1000, &NodeClass::messageCallbackStreamIN, node_class);


  ros::spin();


  return 0;
}

