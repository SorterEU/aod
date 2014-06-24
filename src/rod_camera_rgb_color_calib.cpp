#include "NodeClass.h"
#include <aod/ColorCalibConfig.h>
#include "ColorCalibrationDll_include.h"// Color Calibration

class ColorCalib {
public:
	ColorCalib(){
        sub_=nh_.subscribe("/rod/camera/rgb/image_color", 1, &ColorCalib::messageCallbackImageRectColor, this);
		pub_message_ = nh_.advertise<sensor_msgs::Image>("/rod/camera/rgb/image_calibrated_color", 1000);
	}
	~ColorCalib(){}
	
	void callback(aod::ColorCalibConfig &config, uint32_t level)
	{ 
	 
	  ROS_INFO("Reconfigure request, parameter: %f",
               config.parameter);
	}
	
    void messageCallbackImageRectColor( const sensor_msgs::ImageConstPtr& msg)//const sensor_msgs::Image::ConstPtr& msg)
	{
        ROS_INFO("sensor_msgs::Image");
        //todo


        cv::Mat src;
        float calib_corr_R, calib_corr_G, calib_corr_B;
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

            //Test
    //    cv::Mat roi(src, cv::Rect(10,10,100,100));
    //    cv::Mat roi2(src, cv::Rect(100,100,900,800));
    //    cv::namedWindow("ROI"); //create a display window
    //    cv::imshow("ROI", roi2);
    //    cv::waitKey(1);
            //Test

        myColorCalib::col_cal_compute(src, cv_ptr->image, calib_corr_R, calib_corr_G, calib_corr_B,true);
        myColorCalib::col_cal_use(src,cv_ptr->image,calib_corr_R*0.6,calib_corr_G*0.6,calib_corr_B*0.6,true);

    //        //Test
    //    cv::namedWindow("Calibrated"); //create a display window
    //    cv::imshow("Calibrated", cv_ptr->image);
    //    cv::waitKey(3);//3
    //        //~Test

//        publishMessageImageCalibratedColor(&pub_message_,*(cv_ptr->toImageMsg()));
        pub_message_.publish(*(cv_ptr->toImageMsg()));
		
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
