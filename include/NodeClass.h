#ifndef SR_NODE_CLASS_H
#define SR_NODE_CLASS_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "sorter_msgs/streamOUT.h"

using std::string;

class NodeClass
{
public:

	ros::Publisher  _pub_message;
  //! Constructor.
	NodeClass();

  //! Destructor.
  ~NodeClass();

  //! Publish the message.
  void publishMessage();

  void publishMessage(ros::Publisher *pub_message);
  void publishMessageImageCalibratedColor(ros::Publisher *pub_message);
  void publishMessageTF(ros::Publisher *pub_message);
  void publishMessageImageRectMono(ros::Publisher *pub_message);
  void publishMessageStreamIN(ros::Publisher *pub_message);
  void publishMessageStreamOUT(ros::Publisher *pub_message);

  //! Callback function for subscriber.
 
  //void messageCallback(const node_example::node_example_data::ConstPtr &msg);
  void messageCallback( const std_msgs::String::ConstPtr& msg);
  void messageCallbackImageCalibratedColor( const std_msgs::String::ConstPtr& msg);
  void messageCallbackTF( const std_msgs::String::ConstPtr& msg);
  void messageCallbackImageRectMono( const std_msgs::String::ConstPtr& msg);
  void messageCallbackStreamIN( const std_msgs::String::ConstPtr& msg);
  void messageCallbackStreamOUT( const sorter_msgs::streamOUT::ConstPtr& msg);

};

#endif // SR_NODE_CLASS_H
