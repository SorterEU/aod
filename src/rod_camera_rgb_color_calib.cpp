#include "NodeClass.h"
#include <aod/ColorCalibConfig.h>
#include "ColorCalibrationDll_include.h"// Color Calibration


class ColorCalib {
public:
    double calib_corr_R, calib_corr_G, calib_corr_B;
    bool auto_calib;
    double calib_correction;
    double comp_calib_corr_R, comp_calib_corr_G, comp_calib_corr_B;
    ColorCalib() {
        sub_=nh_.subscribe("/rod/camera/rgb/image_color", 1, &ColorCalib::messageCallbackImageRectColor, this);
		pub_message_ = nh_.advertise<sensor_msgs::Image>("/rod/camera/rgb/image_calibrated_color", 1000);
	}
    ~ColorCalib(){}
	
    void callback(aod::ColorCalibConfig &config, uint32_t level)
	{ 
        if(config.auto_calib==false)
        {
            config.calib_corr_R=comp_calib_corr_R;
            config.calib_corr_G=comp_calib_corr_G;
            config.calib_corr_B=comp_calib_corr_B;
        }
        calib_corr_R = config.calib_corr_R;
        calib_corr_G = config.calib_corr_G;
        calib_corr_B = config.calib_corr_B;
        auto_calib = config.auto_calib;
        calib_correction = config.calib_correction;
	  ROS_INFO("Reconfigure request, parameter: %f",
               config.calib_corr_R,
               config.calib_corr_R,
               config.calib_corr_R,
               config.auto_calib,
               config.calib_correction);
	}


    void messageCallbackImageRectColor( const sensor_msgs::Image::ConstPtr& msg)//const sensor_msgs::ImageConstPtr& msg)
	{
        ROS_INFO("sensor_msgs::Image");
        //todo

        cv::Mat src;
        //float calib_corr_R, calib_corr_G, calib_corr_B;
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        src=cv_ptr->image;

        myColorCalib::col_cal_compute(src, cv_ptr->image, comp_calib_corr_R, comp_calib_corr_G, comp_calib_corr_B, auto_calib);//true);
        myColorCalib::col_cal_use(src,cv_ptr->image,calib_corr_R*calib_correction,calib_corr_G*calib_correction,calib_corr_B*calib_correction, auto_calib);//true);

        pub_message_.publish((cv_ptr->toImageMsg()));
		
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

  //
  // setup params from ParameterServer and store them in dynamic config
  //
  aod::ColorCalibConfig config = aod::ColorCalibConfig::__getDefault__();
  n.param<bool>("auto_calib", config.auto_calib, false);
//  n.param<double>("calib_corr_R", config.calib_corr_R, config.calib_corr_R);
//  n.param<double>("calib_corr_G", config.calib_corr_G, config.calib_corr_G);
//  n.param<double>("calib_corr_B", config.calib_corr_B, config.calib_corr_B);
  //

  dynamic_reconfigure::Server<aod::ColorCalibConfig> srv;
  dynamic_reconfigure::Server<aod::ColorCalibConfig>::CallbackType f;

  ColorCalib colorCalib_;

  n.param<double>("calib_corr_R", config.calib_corr_R, colorCalib_.calib_corr_R);
  n.param<double>("calib_corr_G", config.calib_corr_G, colorCalib_.calib_corr_G);
  n.param<double>("calib_corr_B", config.calib_corr_B, colorCalib_.calib_corr_B);

  f = boost::bind(&ColorCalib::callback, &colorCalib_, _1, _2);
  srv.updateConfig(config);//dodane
  srv.setCallback(f);
    
  ros::Rate loop_rate(10);
  ROS_INFO("Starting to spin...");
  ros::spin();


  return 0;
}
