#include "tic_toc.hpp"
#include <opencv2/core/core.hpp>

double TicToc::t = 0.0 ;

void TicToc::tic()
{
	t = (double) cv::getTickCount();
}	

double TicToc::toc()
{
	return ((double) cv::getTickCount() - t) / cv::getTickFrequency();
}
