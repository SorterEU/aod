#include "ros/ros.h"
#include "std_msgs/String.h"
#include "NodeClass.h"
#include "sorter_msgs/streamOUT.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "cod");
  ros::NodeHandle n;
  
  //Create a new NodeClass object
  NodeClass *node_class = new NodeClass();
   
  //subscribes
  ros::Subscriber sub = n.subscribe("/rod/analysis/streamOUT", 1000, &NodeClass::messageCallbackStreamOUT, node_class);

  ros::spin();

  return 0;
}
