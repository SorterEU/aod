
#include "NodeClass.h"
#include <aod/ImageAggregationConfig.h>


class ImageAggregation {
public:
	ImageAggregation(){
		sub_color_calib_ = nh_.subscribe("/rod/camera/rgb/image_calibrated_color", 1000, &ImageAggregation::messageCallbackImageCalibratedColor, this);
		sub_rgb_STFP_= nh_.subscribe("/TF", 1000, &ImageAggregation::messageCallbackTF, this);
		sub_ir_STFP_= nh_.subscribe("/TF", 1000, &ImageAggregation::messageCallbackTF, this);
		sub_image_proc_= nh_.subscribe("/rod/camera/ir/image_mono", 1000, &ImageAggregation::messageCallbackImageRectMono, this);
		  
		pub_message_ = nh_.advertise<std_msgs::String>("/rod/analysis/streamIN", 1000);
	}
	~ImageAggregation(){}
	
	void callback(aod::ImageAggregationConfig &config, uint32_t level)
	{ 
	 
	  ROS_INFO("Reconfigure request, parameter: %f",
	           config.parameter);
	}
	
	void messageCallbackTF( const std_msgs::String::ConstPtr& msg)
	{
		ROS_INFO("message in messageCallbackTF");
		
		std_msgs::String tmp_msg_;
		std::stringstream ss;
		ss << "StreamIN ";
		tmp_msg_.data = ss.str();
		ROS_INFO("%s", tmp_msg_.data.c_str());

		pub_message_.publish(tmp_msg_);

	}
	
	void messageCallbackImageRectMono( const sensor_msgs::Image::ConstPtr& msg)
	{
		ROS_INFO("message in messageCallbackImageRectMono");
		
		std_msgs::String tmp_msg_;
		std::stringstream ss;
		ss << "StreamIN ";
		tmp_msg_.data = ss.str();
		ROS_INFO("%s", tmp_msg_.data.c_str());

		pub_message_.publish(tmp_msg_);

	}
	
	void messageCallbackImageCalibratedColor( const sensor_msgs::Image::ConstPtr& msg)
	{
		ROS_INFO("message in messageCallbackImageCalibratedColor");
		
		std_msgs::String tmp_msg_;
		std::stringstream ss;
		ss << "StreamIN ";
		tmp_msg_.data = ss.str();
		ROS_INFO("%s", tmp_msg_.data.c_str());

		pub_message_.publish(tmp_msg_);

	}
	
	
	
private:
	ros::Publisher pub_message_;
	ros::NodeHandle nh_;
	ros::Subscriber sub_image_proc_;
	ros::Subscriber sub_ir_STFP_;
	ros::Subscriber sub_rgb_STFP_;
	ros::Subscriber sub_color_calib_;
};

	


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "image_aggregation");
  ros::NodeHandle n;
  
  dynamic_reconfigure::Server<aod::ImageAggregationConfig> srv;
  dynamic_reconfigure::Server<aod::ImageAggregationConfig>::CallbackType f;
    
  ImageAggregation imageAggregation_;
  
  f = boost::bind(&ImageAggregation::callback, &imageAggregation_, _1, _2);
  srv.setCallback(f);
    
  ros::Rate loop_rate(10);
  ROS_INFO("Starting to spin...");
  ros::spin();

  return 0;
}
