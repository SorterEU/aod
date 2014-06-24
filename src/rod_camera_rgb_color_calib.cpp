#include "NodeClass.h"
#include <aod/ColorCalibConfig.h>

class ColorCalib {
public:
	ColorCalib(){
		sub_=nh_.subscribe("/rod/camera/rgb/image_color", 1000, &ColorCalib::messageCallbackImageRectColor, this);
		pub_message_ = nh_.advertise<sensor_msgs::Image>("/rod/camera/rgb/image_calibrated_color", 1000);
	}
	~ColorCalib(){}
	
	void callback(aod::ColorCalibConfig &config, uint32_t level)
	{ 
	 
	  ROS_INFO("Reconfigure request, parameter: %f",
	           config.parameter);
	}
	
	void messageCallbackImageRectColor( const sensor_msgs::Image::ConstPtr& msg)
	{
		ROS_INFO("sensor_msgs::Image");
		//todo
		
	}
		
	
private:
	ros::Publisher pub_message_;
	ros::NodeHandle nh_;
	ros::Subscriber sub_;
};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "color_calib");
  ros::NodeHandle n;

  dynamic_reconfigure::Server<aod::ColorCalibConfig> srv;
  dynamic_reconfigure::Server<aod::ColorCalibConfig>::CallbackType f;

  ColorCalib colorCalib_;
  
  f = boost::bind(&ColorCalib::callback, &colorCalib_, _1, _2);
  srv.setCallback(f);
    
  ros::Rate loop_rate(10);
  ROS_INFO("Starting to spin...");
  ros::spin();


  return 0;
}
