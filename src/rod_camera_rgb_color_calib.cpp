
#include "NodeClass.h"

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "color_calib");
  ros::NodeHandle n;

  //Create a new NodeClass object
  NodeClass *node_class = new NodeClass();

  // Create a publisher and name the topic.
  node_class->_pub_message = n.advertise<sensor_msgs::Image>("/rod/camera/rgb/image_calibrated_color", 1000);

  // Tell ROS how fast to run this node.
  ros::Rate loop_rate(10);
  
  //subscribes
  ros::Subscriber sub_image_proc = n.subscribe("/rod/camera/rgb/image_color", 1000, &NodeClass::messageCallbackImageRectColor, node_class);
  
  ros::spin();


  return 0;
}//end main()
