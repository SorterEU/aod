#include "ros/ros.h"
#include "std_msgs/String.h"
#include "NodeClass.h"

#include <sstream>


//globals:

void image_agregation();


void image_agregation(){}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "image_aggregation");
  ros::NodeHandle n;
  
  //Create a new NodeClass object
  NodeClass *node_class = new NodeClass();
  
  // Create a publisher and name the topic.
  node_class->_pub_message = n.advertise<std_msgs::String>("/rod/analysis/streamIN", 1000);

  // Tell ROS how fast to run this node.
  ros::Rate loop_rate(10); 
  
  //subscribes
  ros::Subscriber sub_color_calib = n.subscribe("/rod/camera/rgb/image_calibrated_color", 1000, &NodeClass::messageCallbackImageCalibratedColor, node_class);
  ros::Subscriber sub_rgb_STFP= n.subscribe("/TF", 1000, &NodeClass::messageCallbackTF, node_class);
  ros::Subscriber sub_ir_STFP= n.subscribe("/TF", 1000, &NodeClass::messageCallbackTF, node_class);
  ros::Subscriber sub_image_proc= n.subscribe("/rod/camera/ir/image_rect_mono", 1000, &NodeClass::messageCallbackImageRectMono, node_class);
  
  
  ros::spin();



  return 0;
}
