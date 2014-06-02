#include "NodeClass.h"

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "STFP");
  ros::NodeHandle n;

  //Create a new NodeClass object
  NodeClass *node_class = new NodeClass();

  // Create a publisher and name the topic.
  ros::Publisher pub_message = n.advertise<std_msgs::String>("/TF", 1000);

  // Tell ROS how fast to run this node.
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
  	  node_class->publishMessageTF(&pub_message);
      ros::spinOnce();
      loop_rate.sleep();
  }


  return 0;
}
