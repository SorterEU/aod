/*
 * ColorCalibrationDll.cpp
 *
 *  Created on: 16 cze 2014
 *      Author: jan.figat
 */

#include <opencv2/opencv.hpp>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;
using namespace std;

//template <typename T> string tostr(const T& t) { ostringstream os; os<<t; return os.str(); }

namespace myColorCalib {
void col_cal_compute( cv::Mat src,cv::Mat& out,double& calib_corr_R,double& calib_corr_G,double& calib_corr_B,bool AutoCalib_On)
{
  if( AutoCalib_On==true )
    {
        vector<Mat> bgr_picture(3); //white balance ; Histogram Equalization R,G,B
        vector<Mat> ycbcr_picture(3); //white balance ; Y, Cb, Cr
        double Y_max; //max Y in YCbCr color space
        float R_avg, G_avg, B_avg;//mean
        Scalar tempVal;
        cv::Mat ycbcr;


//        Rect region_of_interest = Rect(x, y, w, h);
//        cv::Mat image_roi = image(region_of_interest);

        cvtColor(src, ycbcr, CV_RGB2YCrCb);
        split(src, bgr_picture);
        split(ycbcr, ycbcr_picture);
  ///------------------------ HISTOGRAM RGB -----------------------------------

        tempVal = mean( src ); B_avg = floor(tempVal.val[0]); G_avg = floor(tempVal.val[1]); R_avg = floor(tempVal.val[2]);
        //R_avg = floor(tempVal.val[0]); G_avg = floor(tempVal.val[1]); B_avg = floor(tempVal.val[2]);

        minMaxLoc(ycbcr_picture[0],0,&Y_max,0,0,Mat()); //Y channel

        //// White Patches in YCbCr Color Space
        calib_corr_R =Y_max/R_avg; calib_corr_G = Y_max/G_avg; calib_corr_B = Y_max/B_avg;

/*
        bgr_picture[2] *= calib_corr_R;
        bgr_picture[1] *= calib_corr_G;
        bgr_picture[0] *= calib_corr_B;
        merge(bgr_picture , out);
*/
//        out=src;
    }//~if(AutoCalib_On==1)
  else
    {
     out=src;
     calib_corr_B=1.0;
     calib_corr_G=1.0;
     calib_corr_R=1.0;
    }

}

void col_cal_use( cv::Mat src,cv::Mat& out,double calib_corr_R,double calib_corr_G,double calib_corr_B,bool AutoCalib_On)
{
  vector<Mat> bgr_picture(3);
  if( AutoCalib_On==true )
    {
      split(src, bgr_picture);
      bgr_picture[2] *= calib_corr_R;
      bgr_picture[1] *= calib_corr_G;
      bgr_picture[0] *= calib_corr_B;
      merge(bgr_picture , out);
    }
  else
    {
      out=src;
    }
}

}//myColorCalib
