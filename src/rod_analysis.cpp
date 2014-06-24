#include "NodeClass.h"
#include <aod/AnalysisConfig.h>

class Analysis {
public:
	Analysis(){
		sub_=nh_.subscribe("/rod/analysis/streamIN", 1000, &Analysis::messageCallbackStreamIN, this);
		pub_message_ = nh_.advertise<sorter_msgs::streamOUT>("/rod/analysis/streamOUT", 1000);
	}
	~Analysis(){}
	
	void callback(aod::AnalysisConfig &config, uint32_t level)
	{ 
	 
	  ROS_INFO("Reconfigure request, parameter: %f",
	           config.parameter);
	}
	
	void messageCallbackStreamIN( const std_msgs::String::ConstPtr& msg)
	{
		ROS_INFO("message in messageCallbackTF");
		
		sorter_msgs::streamOUT tmp_msg;
		std::stringstream ss;
		ss << "StreamOUT ";
		tmp_msg.message = ss.str();
		ROS_INFO("%s", tmp_msg.message.c_str());
		pub_message_.publish(tmp_msg);
	}
	
private:
	ros::Publisher pub_message_;
	ros::NodeHandle nh_;
	ros::Subscriber sub_;
};




int main(int argc, char **argv)
{

  ros::init(argc, argv, "analysis");
  ros::NodeHandle n;

  dynamic_reconfigure::Server<aod::AnalysisConfig> srv;
  dynamic_reconfigure::Server<aod::AnalysisConfig>::CallbackType f;
  
  Analysis analysis_;
  
  f = boost::bind(&Analysis::callback, &analysis_, _1, _2);
  srv.setCallback(f);
  
  ros::Rate loop_rate(10);
  ROS_INFO("Starting to spin...");
  ros::spin();

  return 0;
}

