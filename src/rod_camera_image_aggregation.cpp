
#include "NodeClass.h"
#include <aod/image_aggregationConfig.h>
#include <sorter_msgs/streamIN.h>


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>

bool trig_ir = false;
bool trig_rgb = false;

ros::Rate * rate = NULL;

ros::Publisher pub_message_;
ros::Publisher pub_debug_[16];

sorter_msgs::streamIN stream_msg;

class ImageAggregation {
public:
	ImageAggregation(){
		sub_color_calib_ = nh_.subscribe("/rod/camera/rgb/image_rect_color", 1, &ImageAggregation::messageCallbackImageCalibratedColor, this);
		sub_rgb_STFP_= nh_.subscribe("/TF", 1000, &ImageAggregation::messageCallbackTF, this);
		sub_ir_STFP_= nh_.subscribe("/TF", 1000, &ImageAggregation::messageCallbackTF, this);
	        sub_image_proc_= nh_.subscribe("/rod/camera/ir/image_rect", 1, &ImageAggregation::messageCallbackImageRectMono, this);
		  
		pub_message_ = nh_.advertise<sorter_msgs::streamIN>("/rod/analysis/streamIN", 1000);
		char buf[256];
		for (int i = 0 ; i < 8; ++i) {
			sprintf(buf, "/rod/analysis/debug_rgb_%d", i);
			pub_debug_[i] = nh_.advertise<sensor_msgs::Image>(buf, 1);
			sprintf(buf, "/rod/analysis/debug_ir_%d", i);
			pub_debug_[i+8] = nh_.advertise<sensor_msgs::Image>(buf, 1);
		}
	}
	~ImageAggregation(){}
	
	void callback(aod::image_aggregationConfig &config, uint32_t level)
	{ 
	 
	  ROS_INFO("Reconfigure request, parameter: %f", config.rate);
		if (rate) delete rate;
		rate = new ros::Rate(config.rate);
	}
	
	sensor_msgs::Image getROI(const sensor_msgs::Image::ConstPtr& msg, cv::Rect roi, const std::string & enc)
	{
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg, enc);

		cv::Mat img = cv_ptr->image(roi);
		cv_ptr->image = img.clone();

		// Output modified video stream
		return *(cv_ptr->toImageMsg());
	}

	void messageCallbackTF( const std_msgs::String::ConstPtr& msg)
	{
		ROS_INFO("message in messageCallbackTF");
	}
	
	void messageCallbackImageRectMono( const sensor_msgs::Image::ConstPtr& msg)
	{
		cv::Rect roi(439, 439, 200, 200);
		ROS_INFO("message in messageCallbackImageRectMono");
		trig_ir = false;
		sensor_msgs::Image img = getROI(msg, roi, "mono8");
		if (stream_msg.ir.size() < 8)
			stream_msg.ir.push_back(img);
		pub_debug_[8+stream_msg.ir.size()-1].publish(img);
	}
	
	void messageCallbackImageCalibratedColor( const sensor_msgs::Image::ConstPtr& msg)
	{
		cv::Rect roi(531, 468, 200, 200);
		ROS_INFO("message in messageCallbackImageCalibratedColor");
		trig_rgb = false;
		sensor_msgs::Image img = getROI(msg, roi, "bgr8");
		if (stream_msg.rgb.size() < 8)
			stream_msg.rgb.push_back(img);
		pub_debug_[stream_msg.rgb.size()-1].publish(img);
	}
	
	
	
private:
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
  
  dynamic_reconfigure::Server<aod::image_aggregationConfig> srv;
  dynamic_reconfigure::Server<aod::image_aggregationConfig>::CallbackType f;
    
  ImageAggregation imageAggregation_;
  
  f = boost::bind(&ImageAggregation::callback, &imageAggregation_, _1, _2);
  srv.setCallback(f);
    
  if (rate) delete rate;
  rate = new ros::Rate(2);
  
  ROS_INFO("Starting to spin...");
  while (ros::ok()) {
  	ros::spinOnce();
  	rate->sleep();
  	trig_ir = true;
  	trig_rgb = true;
  	
  	// check if all images are gathered
  	if (stream_msg.ir.size() >= 8 && stream_msg.rgb.size() >= 8) {
  		pub_message_.publish(stream_msg);
  		stream_msg.ir.clear();
  		stream_msg.rgb.clear();
  		ROS_WARN("Sending streamIN");
  	}
  }

  return 0;
}
