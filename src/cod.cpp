
#include "NodeClass.h"
#include <aod/CodConfig.h>


class Cod{
public:
	Cod(){
		sub_ = nh_.subscribe("/rod/analysis/streamOUT", 1000, &Cod::messageCallbackStreamOUT, this);
	}
	~Cod(){}
	
	void callback(aod::CodConfig &config, uint32_t level)
	{ 
	 
	  ROS_INFO("Reconfigure request, parameter: %f",
	           config.parameter);
	}

	void messageCallbackStreamOUT( const sorter_msgs::streamOUT::ConstPtr& msg)
	{
		ROS_INFO("StreamOut received, message: %s", msg->message.c_str());
	}
private:
	ros::NodeHandle nh_;
	ros::Subscriber sub_;
};



int main(int argc, char **argv)
{

  ros::init(argc, argv, "cod");
  ros::NodeHandle n;
  
  dynamic_reconfigure::Server<aod::CodConfig> srv;
  dynamic_reconfigure::Server<aod::CodConfig>::CallbackType f;
  
  Cod cod_;
  
  f = boost::bind(&Cod::callback, &cod_, _1, _2);
  srv.setCallback(f);
  ros::Rate loop_rate(10);
  
  ROS_INFO("Starting to spin...");
  ros::spin();

  return 0;
}



