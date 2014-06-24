/*
 * ColorCalibrationDll.hpp
 *
 *  Created on: 16 cze 2014
 *      Author: jan.figat
 */

#ifndef COLORCALIBRATIONDLL_HPP_
#define COLORCALIBRATIONDLL_HPP_


namespace myColorCalib {
void col_cal_compute(cv::Mat src,cv::Mat& out,float& calib_corr_R,float& calib_corr_G,float& calib_corr_B,bool AutoCalib_On);
void col_cal_use(cv::Mat src,cv::Mat& out,float calib_corr_R,float calib_corr_G,float calib_corr_B,bool AutoCalib_On);
}


#endif /* COLORCALIBRATIONDLL_HPP_ */
