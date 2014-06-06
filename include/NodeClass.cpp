#include "NodeClass.h"

/*--------------------------------------------------------------------
 * NodeClass()
 * Constructor.
 *------------------------------------------------------------------*/

NodeClass::NodeClass()
{
}

NodeClass::~NodeClass()
{
} 


/*--------------------------------------------------------------------
 * publishMessage()
 * Publish the message.
 *------------------------------------------------------------------*/

void NodeClass::publishMessage()
{

	std_msgs::String msg;
	std::stringstream ss;
	ss << "publishMessage ";
	msg.data = ss.str();

	ROS_INFO("%s", msg.data.c_str());
	_pub_message.publish(msg);

} // end publishMessage()

/*--------------------------------------------------------------------
 * publishMessage()
 * Publish the message.
 *------------------------------------------------------------------*/

void NodeClass::publishMessage(ros::Publisher *pub_message)
{
	
	std_msgs::String msg;
	std::stringstream ss;
	ss << "publishMessage ";
	msg.data = ss.str();

	ROS_INFO("%s", msg.data.c_str());

	pub_message->publish(msg);

} // end publishMessage()

void NodeClass::publishMessageImageCalibratedColor(ros::Publisher *pub_message)
{

	std_msgs::String msg;
	std::stringstream ss;
	ss << "ImageCalibratedColor ";
	msg.data = ss.str();
	ROS_INFO("%s", msg.data.c_str());

	pub_message->publish(msg);	
	
} // end publishMessage()

void NodeClass::publishMessageTF(ros::Publisher *pub_message)
{

	std_msgs::String msg;
	std::stringstream ss;
	ss << "TF ";
	msg.data = ss.str();
	ROS_INFO("%s", msg.data.c_str());

	pub_message->publish(msg);

} // end publishMessage()

void NodeClass::publishMessageImageRectMono(ros::Publisher *pub_message)
{

	std_msgs::String msg;
	std::stringstream ss;
	ss << "ImageRectMono ";
	msg.data = ss.str();
	ROS_INFO("%s", msg.data.c_str());

	pub_message->publish(msg);

} // end publishMessageImageRectMono()


void NodeClass::publishMessageStreamIN(ros::Publisher *pub_message)
{

	std_msgs::String msg;
	std::stringstream ss;
	ss << "StreamIN ";
	msg.data = ss.str();
	ROS_INFO("%s", msg.data.c_str());

	pub_message->publish(msg);

} // end publishMessageStreamIN()

void NodeClass::publishMessageStreamOUT(ros::Publisher *pub_message)
{

	std_msgs::String msg;
	std::stringstream ss;
	ss << "StreamOUT ";
	msg.data = ss.str();
	ROS_INFO("%s", msg.data.c_str());

	pub_message->publish(msg);

} // end publishMessageStreamOUT()


/*-------------------------------------------------------------------*/
//Callbacks
/*--------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *------------------------------------------------------------------*/
void NodeClass::messageCallback( const std_msgs::String::ConstPtr& msg)
{
  //message = msg->message;
  //a = msg->a;
  //b = msg->b;

  // Note that these are only set to INFO so they will print to a terminal for example purposes.
  // Typically, they should be DEBUG.
  ROS_INFO("message in messageCallback");
  
  //ROS_INFO("message in NodeClass", message.c_str());
  //ROS_INFO("sum of a + b = %d", a + b);
} // end messageCallback()

void NodeClass::messageCallbackImageCalibratedColor( const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("message in messageCallbackImageCalibratedColor");
	//todo
	
	publishMessageStreamIN(&_pub_message);

} // end messageCallbackImageCalibratedColor()

void NodeClass::messageCallbackTF( const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("message in messageCallbackTF");
	//todo
	
	publishMessageStreamIN(&_pub_message);

} // end messageCallbackTF()

void NodeClass::messageCallbackImageRectMono( const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("message in messageCallbackImageRectMono");
	//todo
	
	publishMessageStreamIN(&_pub_message);

} // end messageCallbackImageRectMono()


void NodeClass::messageCallbackStreamIN( const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("message in messageCallbackTF");
	//todo
	
	publishMessageStreamOUT(&_pub_message);

} // end messageCallbackStreamIN()

void NodeClass::messageCallbackStreamOUT( const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("message in messageCallbackStreamOUT");
	//todo
	

} // end messageCallbackStreamOUT()
