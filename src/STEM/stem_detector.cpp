#include "stem_detector.hpp"
#include "tic_toc.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <math.h>
#include <limits>

#include <opencv2/highgui/highgui.hpp>

StemDetector::~StemDetector() {} 

void StemDetector::computeScaleSpace(cv::Mat &inputImage, float sigma_begin, float sigma_end, float sigma_log_step, 
									 std::vector<float> &sigma, std::vector<cv::Mat> &Ls, std::vector<cv::Mat> &Rs, 
									 std::vector<cv::Mat> &Ixs, std::vector<cv::Mat> &Iys, bool computeBlobs) 
{
	sigma.clear() ;
	for (float k = 0.0; sigma_begin * exp(k) <= sigma_end ; k += sigma_log_step)
		sigma.push_back(sigma_begin * exp(k)) ;

	//Convert image into float
	cv::Mat inputImageReal(inputImage.size(), CV_32FC1) ;	
	inputImage.convertTo(inputImageReal, CV_32FC1) ;	
	inputImageReal = inputImageReal / 255.0f ;

	//Construct temporary arrays
	cv::Mat Ismooth = inputImageReal.clone() ;	
	cv::Mat Ixx(inputImage.size(), CV_32FC1) ;	
	cv::Mat Iyy(inputImage.size(), CV_32FC1) ;	
	cv::Mat Ixy(inputImage.size(), CV_32FC1) ;	

	//Gamma
	float gamma = 3.0/4.0 ;

	//Clear current scalespace
	Ls.clear() ;
	Rs.clear() ;
	Ixs.clear() ;
	Iys.clear() ;

	float prev_sigma = 0.0f ;
	for (size_t i = 0; i < sigma.size() ; i++) {
		//Prepare output arrays
		cv::Mat L(inputImage.size(), CV_32FC1) ;	//TODO: move it to a single initialization
		cv::Mat R(inputImage.size(), CV_32FC1) ;	//TODO: move it to a single initialization	
		cv::Mat Ix(inputImage.size(), CV_32FC1) ;	//TODO: move it to a single initialization	
		cv::Mat Iy(inputImage.size(), CV_32FC1) ;	//TODO: move it to a single initialization	

		//Perform Gaussian smoothing at given scale
		float curr_sigma = sigma[i] ;
		//float sigma_diff = sqrt(curr_sigma * curr_sigma - prev_sigma * prev_sigma) ;
		float kernel_size = 6 * curr_sigma + 3.0 ;
		size_t kernel_size_int = ceil(kernel_size) ;
		if (kernel_size_int % 2 == 0)
			kernel_size_int++ ;	
		//std::cout << "curr_sigma " << curr_sigma << " prev_sigma " << prev_sigma << " sigma_diff " << sigma_diff << " kernel_size_int " << kernel_size_int << std::endl ;
		cv::GaussianBlur(inputImageReal, Ismooth, cv::Size(kernel_size_int, kernel_size_int), curr_sigma) ; //140ms (inceremental - 80ms - different results)
		//cv::Mat Ismooth1(inputImage.size(), CV_32FC1) ;
		//cv::GaussianBlur(inputImageReal, Ismooth1, cv::Size(kernel_size_int, kernel_size_int), curr_sigma) ;
		//cv::Mat diff = Ismooth - Ismooth1 ;
		//displayImage(diff) ;
		//std::cout << "Norm diff = " << cv::norm(diff, cv::NORM_INF) << " norm origin = " << cv::norm(Ismooth1, cv::NORM_INF) << std::endl << " ratio " << cv::norm(diff, cv::NORM_INF) / cv::norm(Ismooth1, cv::NORM_INF) << std::endl ;

		prev_sigma = curr_sigma ;

		//Sobel differential operators
		cv::Sobel(Ismooth, Ixx, CV_32F, 2, 0) ;
		cv::Sobel(Ismooth, Iyy, CV_32F, 0, 2) ;
		cv::Sobel(Ismooth, Ixy, CV_32F, 1, 1) ; //70ms
		cv::Sobel(Ismooth, Ix, CV_32F, 1, 0) ;
		cv::Sobel(Ismooth, Iy, CV_32F, 0, 1) ;

		float t = sigma[i] * sigma[i] ; //scale factor

		//Compute Laplacian
		if (computeBlobs) {
			L = t * (Ixx + Iyy) ; //20ms
			Ls.push_back(L) ;
		}

		//Compute ridge response

		float tgamma = pow(t, gamma) ;

		int nRows = Ixx.rows ;
		int nCols = Ixx.cols ;

		for (int is = 0; is < nRows ; is++) {
			const float* rIxx = Ixx.ptr<float>(is) ;
			const float* rIyy = Iyy.ptr<float>(is) ;
			const float* rIxy = Ixy.ptr<float>(is) ;
			float* rR = R.ptr<float>(is) ;
			for (int js = 0; js < nCols ; js++) {
				float ixx = rIxx[js] ;
				float iyy = rIyy[js] ; 
				float itemp1 = ixx + iyy ; 
				float r = 0.0f ;
				if (itemp1 >= 0) { //Laplacian is grater than 0 - so the strongest curvature is positive
					float ixy = rIxy[js] ; 
					float itemp2 = ixx - iyy ;
					r = tgamma * sqrt(fabs(itemp1) * sqrt(itemp2 * itemp2 + 4 * ixy * ixy)) ;
					//if (r > 0)
					//	std::cout << r ;
				}	
				rR[js] = r ; 
			}
		} //100ms

		Rs.push_back(R) ;
		Ixs.push_back(Ix) ;
		Iys.push_back(Iy) ; 
	}
}

void StemDetector::dilate3DArray(std::vector<cv::Mat> &Arr, std::vector<cv::Mat> &ArrRes, size_t ksize)
{
	//First dilate each layer of our array with square kernel
	cv::Mat rectElement = getStructuringElement(cv::MORPH_RECT, cv::Size(ksize, ksize)); //2-D structural element
	cv::Mat rowElement = getStructuringElement(cv::MORPH_RECT, cv::Size(1, ksize)); //1-D structural element

	//Some constants
	int nRows = Arr[0].rows ;
	int nCols = Arr[0].cols ;
	size_t nPlanes  = Arr.size() ;

	//Prepare output arrays //TODO:move it to a single initialization
	for (size_t k = 0; k < nPlanes ; k++)
		ArrRes.push_back(cv::Mat(inputImageHoles.size(), CV_32FC1)) ;

	//Single plane dilation	
	for (size_t i = 0; i < Arr.size() ; i++) 
		cv::dilate(Arr[i], ArrRes[i], rectElement) ;

	//Let us construct a slice including given row 
	cv::Mat slice(cv::Size(Arr[0].cols, Arr.size()), CV_32FC1) ;

	//Now, perform dilation along the third dimension
	for (int i = 0; i < nRows ; i++) {
		//Copy elements from Arr column to the slice
		for (size_t k = 0; k < nPlanes ; k++) {
			//Now copy a single row to slice
			const float* arrRow = ArrRes[k].ptr<float>(i) ;
			float* sliceRow = slice.ptr<float>(k) ;
			std::copy(arrRow, arrRow + nCols, sliceRow) ;
		}

		//Perform 1D dilation
		cv::dilate(slice, slice, rowElement) ;	

		for (size_t k = 0; k < nPlanes ; k++) {
			//Now copy a single row back from the slice 
			float* arrRow = ArrRes[k].ptr<float>(i) ;
			const float* sliceRow = slice.ptr<float>(k) ;
			std::copy(sliceRow, sliceRow + nCols, arrRow) ;
		}
	}
}	

void StemDetector::suppressNonMax(std::vector<cv::Mat> &Arr, std::vector<cv::Mat> &ArrMax)
{
	//Suppress all results where Arr < ArrMax (non-maxima)
	cv::Mat mask(cv::Size(Arr[0].size()), CV_32FC1) ;
	for (size_t k = 0; k < Arr.size() ; k++) {
		cv::compare(Arr[k], ArrMax[k], mask, cv::CMP_LT) ;
		Arr[k].setTo(0.0f, mask) ;
	}	
}

void StemDetector::computeObjectMask(int threshold, cv::Mat &mask)
{
	//After segment_objects.cpp OpenCV example
	int niters = 3;

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::Mat temp ;
	cv::threshold(inputImageMask, temp, threshold, 255, cv::THRESH_BINARY) ;

	cv::dilate(temp, temp, cv::Mat(), cv::Point(-1,-1), niters);
	cv::erode(temp, temp, cv::Mat(), cv::Point(-1,-1), niters*2);
	cv::dilate(temp, temp, cv::Mat(), cv::Point(-1,-1), niters);

	cv::findContours(temp, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE) ;

	mask = cv::Mat::zeros(inputImageMask.size(), CV_8UC1);

	if( contours.size() == 0 )
		return;

	int idx = 0, largestComp = 0;
	double maxArea = 0;

	for( ; idx >= 0; idx = hierarchy[idx][0] )
	{
		const std::vector<cv::Point>& c = contours[idx];
		double area = fabs(contourArea(cv::Mat(c)));
		if( area > maxArea )
		{
			maxArea = area;
			largestComp = idx;
		}
	}

	cv::Scalar color(255);
	//cv::drawContours(mask, contours, largestComp, color, CV_FILLED, 8, hierarchy ); //Use of hierarchy produces holes in the area
	cv::drawContours(mask, contours, largestComp, color, CV_FILLED, 8);
	
	cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 3);
}

void StemDetector::filterWithBinaryMask(std::vector<cv::Mat> &Arr, cv::Mat &mask)
{
	cv::Mat maskinv = 255 - mask ; //TODO: we may want to compute mask inversion once (for all subsequent operations)
	for (size_t k = 0; k < Arr.size() ; k++) 
		Arr[k].setTo(0.0f, maskinv) ; 
}

void StemDetector::filterWeakMaxima(std::vector<cv::Mat> &Arr, double ratio)
{
	//Search for a global maximum	
	double globalMax = std::numeric_limits<float>::min() ; 
	for (size_t k = 0; k < Arr.size() ; k++) {
		double minVal, maxVal ;
		cv::minMaxLoc(Arr[k], &minVal, &maxVal) ;
		globalMax = std::max(maxVal, globalMax) ;
	}

	//Now suppress all maxima significantly below the global maximum
	for (size_t k = 0; k < Arr.size() ; k++) 
		cv::threshold(Arr[k], Arr[k], globalMax * ratio, 0.0, cv::THRESH_TOZERO) ;
	
}

void StemDetector::extractMaximaList(std::vector<cv::Mat> &Arr, std::list<MaximumDescriptor> &descriptors)
{
	//Some constants
	int nRows = Arr[0].rows ;
	int nCols = Arr[0].cols ;
	size_t nPlanes  = Arr.size() ;

	//Find all maxima left (non-zero elements)
	descriptors.clear() ;
	for (size_t k = 0; k < nPlanes ; k++) {
		for (int i = 0; i < nRows ; i++) {
			const float* row = Arr[k].ptr<float>(i) ;
			for (int j = 0; j < nCols ; j++)
				if (row[j] > 0)
					descriptors.push_back(MaximumDescriptor(i, j, k, row[j])) ;
		}
	}
}

void StemDetector::describeBlobs(std::list<MaximumDescriptor> &mdescriptors, std::list<FeatureDescriptor> &fdescriptors, std::vector<float> &sigma)
{
	fdescriptors.clear() ;	
	for(std::list<MaximumDescriptor>::iterator it = mdescriptors.begin() ; it != mdescriptors.end() ; it++) {
		FeatureDescriptor fd ;
		fd.pos = (cv::Mat_<float>(2,1) << it->j, it->i) ; 
		fd.extent =  cv::Mat::eye(2, 2, CV_32F) * sigma[it->k] * sqrt(2.0f) ;
		fd.response = it->val * blobRidgeResponseRatio ;
		fdescriptors.push_back(fd) ;
	}
}

void StemDetector::describeValleys(std::list<MaximumDescriptor> &mdescriptors, std::vector<float> &sigma, std::vector<cv::Mat> &Ixs, std::vector<cv::Mat> &Iys, std::list<FeatureDescriptor> &fdescriptors) 
{
	fdescriptors.clear() ;
	for(std::list<MaximumDescriptor>::iterator it = mdescriptors.begin() ; it != mdescriptors.end() ; it++) {
		//Compute structure tensor
		cv::Mat Ix = Ixs[it->k] ;
		cv::Mat Iy = Iys[it->k] ;
		float curr_sigma = 3 * sigma[it->k] ;
		float kernel_size = 6 * curr_sigma + 3.0 ;
		size_t kernel_size_int = ceil(kernel_size) ;
		if (kernel_size_int % 2 == 0)
			kernel_size_int++ ;	

		//Prepare gauss kernel (normalizing coefficient is not important - we normalize eigenvalues afterwards)
		cv::Mat gauss_kernel = cv::getGaussianKernel(kernel_size_int, curr_sigma, CV_32F) ;
		cv::mulTransposed(gauss_kernel, gauss_kernel, false);

		int kernel_size_half = kernel_size_int / 2 ;
		int i = it->i ;
		int j = it->j ;
		int mini = std::max(i - kernel_size_half, 0) ;
		int minj = std::max(j - kernel_size_half, 0) ;
		int maxi = std::min(i + kernel_size_half, Ix.rows - 1) ;
		int maxj = std::min(j + kernel_size_half, Ix.cols - 1) ;

		float Ix2cum = 0.0f ;
		float Iy2cum = 0.0f ; 
		float Ixycum = 0.0f ;

		//std::cout << "Gauss kernel size: " << gauss_kernel.rows << " " << gauss_kernel.cols << std::endl ;
		//std::cout << "Gauss kernel: " << gauss_kernel << std::cout ;
		for (int in = mini; in <= maxi ; in++) {
			const float* rowIx = Ix.ptr<float>(in) ;
			const float* rowIy = Iy.ptr<float>(in) ;
			const float* rowKernel = gauss_kernel.ptr<float>(in - i + kernel_size_half) ; 
			for (int jn = minj; jn < maxj ; jn++) {
				//std::cout << "indices: " << in - i + kernel_size_half << " " << jn - j + kernel_size_half << std::endl ;
				//std::cout << "out of: " << kernel_size_int ;
				float ixval = rowIx[jn] ;
				float iyval = rowIy[jn] ;
				float kernelval = rowKernel[jn - j + kernel_size_half] ;
				//std::cout << "elem: " << ixval << " " << iyval << " " << kernelval << std::endl ;
				Ix2cum += ixval * ixval * kernelval ;
				Iy2cum += iyval * iyval * kernelval ;
				Ixycum += ixval * iyval * kernelval ;
			}
		}
		//Eigenvectors scaled by eigenvalues and scale factor form axes of the ellipse
		cv::Mat tensor = (cv::Mat_<float>(2,2) << Ix2cum, Ixycum, Ixycum, Iy2cum) ;
		//std::cout << "tensor " << tensor ;
		cv::Mat eigenvalues, eigenvectors ;
		cv::eigen(tensor, eigenvalues, eigenvectors) ;
		cv::Mat D1 = (cv::Mat_<float>(2,2) << eigenvalues.at<float>(1), 0.0f, 0.0f, eigenvalues.at<float>(0)) ; //Change the precedence of eigenvalues (small eigenvalue denote slow intensity change and a large axis)
		//Due to sorting - the large axis is second and the small axis goes first!!
		float mind = std::min(eigenvalues.at<float>(0), eigenvalues.at<float>(1)) ;
		D1 = D1 / mind ;

		/*std::cout << "Data1: " << it->i << " " << it->j << " " << it->k  << std::endl ;
		std::cout << "Data2: " << mini << " " << minj << " " << maxi  << " " << maxj << " " << std::endl ;
		std::cout << "Data3 " << Ix2cum << " " << Iy2cum << " " << Ixycum << std::endl ;
		std::cout << "tensor:" << std::endl ;
		std::cout << tensor << std::endl ;
		std::cout << "D1:" << std::endl ;
		std::cout << D1 << std::endl ;
		std::cout << "V:" << std::endl ;
		std::cout << eigenvectors << std::endl ;
		std::cout << "Sigma:" << sigma[it->k] << std::endl ; */

		FeatureDescriptor fd ;
		fd.pos = (cv::Mat_<float>(2,1) << it->j, it->i) ; 
		fd.extent = eigenvectors.t() * D1 * sigma[it->k] ; //Beware of transposition - eigenvectors are row vectors here (unlike in Matlab)!
		//std::cout << "Eigenvectors " << eigenvectors << std::endl ;
		//std::cout << eigenvectors.at<float>(0,0) * eigenvectors.at<float>(0,0) + eigenvectors.at<float>(0,1) * eigenvectors.at<float>(0,1) << std::endl ;
		//std::cout << eigenvectors.at<float>(1,0) * eigenvectors.at<float>(1,0) + eigenvectors.at<float>(1,1) * eigenvectors.at<float>(1,1) << std::endl ;
		//std::cout << "Extent " << fd.extent << std::endl ;
		fd.response = it->val ;
		fdescriptors.push_back(fd) ;
		//std::cout << "Extent:" << std::endl ;
		//std::cout << fd.extent << std::endl ;
	}
}

void StemDetector::describeValleys1(std::list<MaximumDescriptor> &mdescriptors, std::vector<float> &sigma, std::vector<cv::Mat> &Rss, std::list<FeatureDescriptor> &fdescriptors) 
{
	fdescriptors.clear() ;
	for(std::list<MaximumDescriptor>::iterator it = mdescriptors.begin() ; it != mdescriptors.end() ; it++) {
		cv::Mat Rs = Rss[it->k] ;
		float curr_sigma = 2.0f * sigma[it->k] ;
		float kernel_size = 6 * curr_sigma + 3.0 ;
		size_t kernel_size_int = ceil(kernel_size) ;
		if (kernel_size_int % 2 == 0)
			kernel_size_int++ ;	

		//Prepare gauss kernel (normalizing coefficient is not important - we normalize eigenvalues afterwards)
		cv::Mat gauss_kernel = cv::getGaussianKernel(kernel_size_int, curr_sigma, CV_32F) ;
		cv::mulTransposed(gauss_kernel, gauss_kernel, false);

		int kernel_size_half = kernel_size_int / 2 ;
		int i = it->i ;
		int j = it->j ;
		int mini = std::max(i - kernel_size_half, 0) ;
		int minj = std::max(j - kernel_size_half, 0) ;
		int maxi = std::min(i + kernel_size_half, Rs.rows - 1) ;
		int maxj = std::min(j + kernel_size_half, Rs.cols - 1) ;

		float Icum = 0.0f ;
		float Jcum = 0.0f ; 
		float IJcum = 0.0f ;

		for (int in = mini; in <= maxi ; in++) {
			const float* rowI = Rs.ptr<float>(in) ;
			const float* rowKernel = gauss_kernel.ptr<float>(in - i + kernel_size_half) ; 
			for (int jn = minj; jn < maxj ; jn++) {
				float val = rowI[jn] ;
				float kernelval = rowKernel[jn - j + kernel_size_half] ;
				//kernelval = 1.0f ;

				Icum += (in - i) * (in - i) * val * kernelval ; 
				Jcum += (jn - j) * (jn - j) * val * kernelval ; 
				IJcum += (in - i) * (jn - j) * val * kernelval ;
			}
		}
		//Eigenvectors scaled by eigenvalues and scale factor form axes of the ellipse
		cv::Mat tensor = (cv::Mat_<float>(2,2) << Jcum, IJcum, IJcum, Icum) ;
		//std::cout << "tensor " << tensor ;
		cv::Mat eigenvalues, eigenvectors ;
		cv::eigen(tensor, eigenvalues, eigenvectors) ;
		cv::Mat D1 = (cv::Mat_<float>(2,2) << eigenvalues.at<float>(1), 0.0f, 0.0f, eigenvalues.at<float>(0)) ; //Change the precedence of eigenvalues (small eigenvalue denote slow intensity change and a large axis)
		//Due to sorting - the large axis is second and the small axis goes first!!
		float mind = std::min(eigenvalues.at<float>(0), eigenvalues.at<float>(1)) ;
		D1 = D1 / mind ;

		/*std::cout << "Data1: " << it->i << " " << it->j << " " << it->k  << std::endl ;
		std::cout << "Data2: " << mini << " " << minj << " " << maxi  << " " << maxj << " " << std::endl ;
		std::cout << "Data3 " << Ix2cum << " " << Iy2cum << " " << Ixycum << std::endl ;
		std::cout << "tensor:" << std::endl ;
		std::cout << tensor << std::endl ;
		std::cout << "D1:" << std::endl ;
		std::cout << D1 << std::endl ;
		std::cout << "V:" << std::endl ;
		std::cout << eigenvectors << std::endl ;
		std::cout << "Sigma:" << sigma[it->k] << std::endl ; */

		FeatureDescriptor fd ;
		fd.pos = (cv::Mat_<float>(2,1) << it->j, it->i) ; 
		cv::Mat eigenvectorsswp(2,2,CV_32FC1) ;
		eigenvectors.row(0).copyTo(eigenvectorsswp.row(1)) ;
		eigenvectors.row(1).copyTo(eigenvectorsswp.row(0)) ;
		fd.extent = eigenvectorsswp.t() * D1 * sigma[it->k] ; //Beware of transposition - eigenvectors are row vectors here (unlike in Matlab)!
		//std::cout << "Eigenvectors " << eigenvectors << std::endl ;
		//std::cout << eigenvectors.at<float>(0,0) * eigenvectors.at<float>(0,0) + eigenvectors.at<float>(0,1) * eigenvectors.at<float>(0,1) << std::endl ;
		//std::cout << eigenvectors.at<float>(1,0) * eigenvectors.at<float>(1,0) + eigenvectors.at<float>(1,1) * eigenvectors.at<float>(1,1) << std::endl ;
		//std::cout << "Extent " << fd.extent << std::endl ;
		fd.response = it->val ;
		fdescriptors.push_back(fd) ;
		//std::cout << "Extent:" << std::endl ;
		//std::cout << fd.extent << std::endl ;
	}
}

cv::RotatedRect StemDetector::getRotatedRect(const cv::Mat &pos, const cv::Mat &extent)
{
	float axis1x = extent.at<float>(0,0) ;
	float axis1y = extent.at<float>(1,0) ;

	float axis2x = extent.at<float>(0,1) ;
	float axis2y = extent.at<float>(1,1) ;

	float a = sqrt(axis1x*axis1x+axis1y*axis1y) ;
	float b = sqrt(axis2x*axis2x+axis2y*axis2y) ;

	float posx = pos.at<float>(0) ;
	float posy = pos.at<float>(1) ;

	//Let axis1 be the major axis
	float angle = std::atan2(axis1y, axis1x) / CV_PI * 180.0f ; ;		

	cv::RotatedRect rr(cv::Point2f(posx, posy), cv::Size2f(2 * a, 2 * b), angle) ;

	return rr ;
}

bool StemDetector::isLarge(const FeatureDescriptor &fd)
{
	//a - large axis, b - small axis 
	float axis1x = fd.extent.at<float>(0,0) ;
	float axis1y = fd.extent.at<float>(1,0) ;

	float axis2x = fd.extent.at<float>(0,1) ;
	float axis2y = fd.extent.at<float>(1,1) ;

	float b = sqrt(axis1x*axis1x+axis1y*axis1y) ;
	float a = sqrt(axis2x*axis2x+axis2y*axis2y) ;

	if (a * b * CV_PI > minHoleArea)
		return true ;
	else
		return false ;
} 

bool StemDetector::isOblong(const FeatureDescriptor &fd)
{
	//a - large axis, b - small axis 
	float axis1x = fd.extent.at<float>(0,0) ;
	float axis1y = fd.extent.at<float>(1,0) ;

	float axis2x = fd.extent.at<float>(0,1) ;
	float axis2y = fd.extent.at<float>(1,1) ;

	float b = sqrt(axis1x*axis1x+axis1y*axis1y) ;
	float a = sqrt(axis2x*axis2x+axis2y*axis2y) ;

	//std::cout << fd.extent << std::endl ;
	//std::cout << "a = " << a << "b = " << b ;
	if (a > minStemAxisRatio * b && b < maxStemAxisLength && a > minStemAxisLength) {
		//std::cout << " is stem " << std::endl ;
		return true ;
	} else {
		//std::cout << " is not stem " << std::endl ;
		return false ;
	}
}

void StemDetector::findStems(const std::list<FeatureDescriptor> &features, std::list<FeatureDescriptor> &stems)
{
	stems.clear() ;
	for (std::list<FeatureDescriptor>::const_iterator it = features.begin(); it != features.end() ; it++)
		if (isOblong(*it))
			stems.push_back(*it) ;
}

void StemDetector::findHoles(const std::list<FeatureDescriptor> &features, std::list<FeatureDescriptor> &holes)
{
	holes.clear() ;
	for (std::list<FeatureDescriptor>::const_iterator it = features.begin(); it != features.end() ; it++)
		if (isLarge(*it) && !isOblong(*it))
			holes.push_back(*it) ;
}

bool StemDetector::isStemHoleRelated(const FeatureDescriptor &hole, const FeatureDescriptor &stem)
{
	float xh = hole.pos.at<float>(0) ;
	float yh = hole.pos.at<float>(1) ;

	float xs1 = stem.pos.at<float>(0) + stem.extent.at<float>(0,0) ;
	float ys1 = stem.pos.at<float>(1) + stem.extent.at<float>(1,0) ;

	float xs2 = stem.pos.at<float>(0) + stem.extent.at<float>(0,1) ;
	float ys2 = stem.pos.at<float>(1) + stem.extent.at<float>(1,1) ;

	float l1 = sqrt((xh - xs1) * (xh - xs1) + (yh - ys1) * (yh - ys1)) ;
	float l2 = sqrt((xh - xs2) * (xh - xs2) + (yh - ys2) * (yh - ys2)) ;

	if (l1 < maxStemHoleDist || l2 < maxStemHoleDist)
		return true ;
	else
		return false ;
}

void StemDetector::findPairs(const std::list<FeatureDescriptor> &holes, const std::list<FeatureDescriptor> &stems, std::list<FeatureDescriptor> &pairs)
{
	//Cleanup	
	pairs.clear() ;

	//Find pairs
	for (std::list<FeatureDescriptor>::const_iterator it1 = holes.begin(); it1 != holes.end() ; it1++) //Iterator holes
		for (std::list<FeatureDescriptor>::const_iterator it2 = stems.begin(); it2 != stems.end() ; it2++) //Iterate stems
			if (isStemHoleRelated(*it1, *it2)) {
				//Add a new hole-stem pair
				FeatureDescriptor hole = *it1 ;
				FeatureDescriptor stem = *it2 ;
				FeatureDescriptor pair ; 

				//Saving pair positions
				pair.pos = cv::Mat_<float>(2,2) ;
				it1->pos.copyTo(pair.pos(cv::Rect_<int>(0,0,1,2))) ;
				it2->pos.copyTo(pair.pos(cv::Rect_<int>(1,0,1,2))) ;

				//Saving pair extents
				pair.extent = cv::Mat_<float>(2,4) ;
				it1->extent.copyTo(pair.extent(cv::Rect_<int>(0,0,2,2))) ;
				it2->extent.copyTo(pair.extent(cv::Rect_<int>(2,0,2,2))) ;
				pair.response = (it1->response + it2->response) * selectStemPairRatio ;

				//Debug display
				/*std::cout << "Hole position " << it1->pos << std::endl ;
				std::cout << "Hole extent " << it1->extent << std::endl ;
				std::cout << "Stem position " << it2->pos << std::endl ;
				std::cout << "Stem extent " << it2->extent << std::endl ;
				std::cout << "Pair position " << pair.pos << std::endl ;
				std::cout << "Pair extent " << pair.extent << std::endl ;*/
				
				pairs.push_back(pair) ;
			}	

	//Re-sort hole-stem pairs
	pairs.sort() ;
}

void StemDetector::setInputImages(cv::Mat &inputImageHoles, cv::Mat &inputImageStems, cv::Mat &inputImageMask)
{
	this->inputImageHoles = inputImageHoles ;
	this->inputImageStems = inputImageStems ;
	this->inputImageMask = inputImageMask ;
} 

int StemDetector::detectHoleStem(FeatureDescriptor &selFeature)
{
	TicToc::tic() ;	
	
	computeScaleSpace(inputImageHoles, sigmaHoleStart, sigmaHoleStop, sigmaHoleLogStep, sigmaHole, Lsh, Rsh, Ixsh, Iysh, true) ;
	computeScaleSpace(inputImageStems, sigmaStemStart, sigmaStemStop, sigmaStemLogStep, sigmaStem, Lss, Rss, Ixss, Iyss, false) ;

	//StemDetector::displayImages(Lsh) ;

	std::cout << "Scalespace computation time (s) [" << TicToc::toc() << "]" << std::endl ;

	for (size_t k = 0; k < sigmaHole.size() ; k++)
		std::cout << "k: " << k << ", sigmaHole = " << sigmaHole[k] << std::endl ;

	for (size_t k = 0; k < sigmaStem.size() ; k++)
		std::cout << "k: " << k << ", sigmaStem = " << sigmaStem[k] << std::endl ;

	TicToc::tic() ;	
	std::vector<cv::Mat> Lshmax ;
	std::vector<cv::Mat> Rshmax, Rssmax ;

	dilate3DArray(Lsh, Lshmax, 3) ;
	dilate3DArray(Rsh, Rshmax, 3) ;
	dilate3DArray(Rss, Rssmax, 3) ;
	std::cout << "3D dilation time (s) [" << TicToc::toc() << "]" << std::endl ;

	std::vector<cv::Mat> Rsscopy ;
	for (size_t i = 0; i < Rss.size() ; i++)
		Rsscopy.push_back(Rss[i].clone()) ;


	TicToc::tic() ;	
	suppressNonMax(Lsh, Lshmax) ;
	suppressNonMax(Rsh, Rshmax) ;
	suppressNonMax(Rss, Rssmax) ;
	std::cout << "Non-max suppression time (s) [" << TicToc::toc() << "]" << std::endl ;

	TicToc::tic() ;	
	computeObjectMask(backgroundThreshold, binaryMask) ;
	std::cout << "Binary mask computation (s) [" << TicToc::toc() << "]" << std::endl ;

	TicToc::tic() ;	
	filterWithBinaryMask(Lsh, binaryMask) ;	
	filterWithBinaryMask(Rsh, binaryMask) ;	
	filterWithBinaryMask(Rss, binaryMask) ;	
	std::cout << "Filtering with object mask (s) [" << TicToc::toc() << "]" << std::endl ;

	TicToc::tic() ;	
	filterWeakMaxima(Lsh, weakSuppRatio) ;
	filterWeakMaxima(Rsh, weakSuppRatio) ;
	filterWeakMaxima(Rss, weakSuppRatio) ;
	std::cout << "Filtering weak maxima (s) [" << TicToc::toc() << "]" << std::endl ;

	TicToc::tic() ;	
	extractMaximaList(Lsh, Lhlist) ;
	extractMaximaList(Rsh, Rhlist) ;
	extractMaximaList(Rss, Rslist) ;
	std::cout << "Extracting maxima list(s) [" << TicToc::toc() << "]" << std::endl ;

	std::cout << "Lhlist size = " << Lhlist.size() << std::endl ;
	std::cout << "Rhlist size = " << Rhlist.size() << std::endl ;
	std::cout << "Rslist size = " << Rslist.size() << std::endl ;

	TicToc::tic() ;	
	describeBlobs(Lhlist, LhFlist, sigmaHole) ;
	describeValleys(Rhlist, sigmaHole, Ixsh, Iysh, RhFlist) ;
	//describeValleys(Rslist, sigmaStem, Ixss, Iyss, RsFlist) ;
	//StemDetector::displayImages(Rsscopy) ;
	describeValleys1(Rslist, sigmaStem, Rsscopy, RsFlist) ;
	std::cout << "Describe blobs and valleys (s) [" << TicToc::toc() << "]" << std::endl ;

	//Sort blobs and valleys
	LhFlist.sort() ;
	RhFlist.sort() ;
	RsFlist.sort() ;

	//Merge lists into one (maintaining sorted order)
	hfList.clear() ;
	hfList.merge(std::list<FeatureDescriptor>(LhFlist)) ;
	hfList.merge(std::list<FeatureDescriptor>(RhFlist)) ;

	for(std::list<FeatureDescriptor>::iterator it = hfList.begin(); it != hfList.end() ; it++)
		it->response *= holeStemResponseRatio ;

	sfList.clear() ;
	sfList.merge(std::list<FeatureDescriptor>(RsFlist)) ;

	//Find hole and stem candidates
	findHoles(hfList, holeList) ; 
	findStems(sfList, stemList) ; 

	//Find stem-hole pairs	
	findPairs(holeList, stemList, pairList) ;

	//Merge sorter lists of holes and stem-hole pairs
	holePairList.clear() ;
	holePairList.merge(std::list<FeatureDescriptor>(holeList)) ;
	holePairList.merge(std::list<FeatureDescriptor>(pairList)) ;	

	//Make decision concerning hole and stem	
	int decision = 0 ;	

	if (!holePairList.empty())
		if (holePairList.back().response >= reliableDetectionThreshold) {
			selFeature = holePairList.back() ;
			decision = 1 ; 
		}

	return decision ;
}

void StemDetector::printMaximumDescriptors(std::list<MaximumDescriptor> &mdescriptors, std::vector<float> &sigma)
{
	for(std::list<MaximumDescriptor>::iterator it = mdescriptors.begin() ; it != mdescriptors.end() ; it++) {
		std::cout << "(i, j, sigma) = " << "(" << it->i << ", " << it->j << ", " << sigma[it->k] << ")" << std::endl ;
	}
}

void StemDetector::printFeatureDescriptors(std::list<FeatureDescriptor> &fdescriptors)
{
	for(std::list<FeatureDescriptor>::iterator it = fdescriptors.begin() ; it != fdescriptors.end() ; it++) {
		std::cout << "(pos, extent, response) = " << it->pos << ", " << it->extent << ", " << it->response << std::endl ; 
	}
}

void StemDetector::displayImage(cv::Mat &image, const std::string &windowName)
{
	cv::namedWindow( windowName, cv::WINDOW_AUTOSIZE );
	double minVal, maxVal ;

	cv::minMaxLoc(image, &minVal, &maxVal) ;
	//std::cout << "\n minVal = " << minVal << " maxVal = " << maxVal << std::endl ;

	cv::Mat Idisplay ;
	convertScaleAbs(image, Idisplay, 255.0 / (maxVal - minVal), - 255.0 * minVal / (maxVal - minVal));
	cv::imshow( windowName, Idisplay);                   
	//cv::waitKey(0);     
}

void StemDetector::displayImages(std::vector<cv::Mat> &imspace)
{
	double globalMin = std::numeric_limits<float>::max() ; 
	double globalMax = std::numeric_limits<float>::min() ; 

	for (size_t k = 0; k < imspace.size() ; k++) {
		double minVal, maxVal ;
		cv::minMaxLoc(imspace[k], &minVal, &maxVal) ;
		globalMin = std::min(minVal, globalMin) ;
		globalMax = std::max(maxVal, globalMax) ;
		//std::cout << "\n minVal = " << minVal << " maxVal = " << maxVal << std::endl ;

	}

	//std::cout << "\n minVal = " << globalMin << " maxVal = " << globalMax << std::endl ;
	cv::Mat Idisplay ;
	for (size_t k = 0; k < imspace.size() ; k++) {
		convertScaleAbs(imspace[k], Idisplay, 255.0 / (globalMax - globalMin), - 255.0 * globalMin / (globalMax - globalMin));
		cv::imshow( "Display window", Idisplay);                   
		cv::waitKey(0) ;     
	}

}

void StemDetector::drawFeature(cv::Mat &image, const FeatureDescriptor &fdesc, cv::Scalar color)
{
	//std::cout << "All pos: " << fdesc.pos << std::endl ;
	//std::cout << "All extent: " << fdesc.extent << std::endl ;
	for (int j = 0; j < fdesc.pos.cols ; j++) {
		cv::Mat pos, extent ;
		pos = cv::Mat_<float>(2,1) ;
		extent = cv::Mat_<float>(2,2) ;

		fdesc.pos(cv::Rect_<int>(j,0,1,2)).copyTo(pos) ;
		fdesc.extent(cv::Rect_<int>(j * 2,0,2,2)).copyTo(extent) ;

		//std::cout << "Sel. pos: " << pos << std::endl ;
		//std::cout << "Sel. extent: " << extent << std::endl ;

		cv::RotatedRect rr = getRotatedRect(pos, extent) ;
		cv::ellipse(image, rr, color) ;
	}
}

void StemDetector::drawFeatures(cv::Mat &image, std::list<FeatureDescriptor> &fdescriptors, cv::Scalar color)
{
	for(std::list<FeatureDescriptor>::iterator it = fdescriptors.begin() ; it != fdescriptors.end() ; it++) {
		drawFeature(image, *it, color) ;
		//cv::RotatedRect rr = getRotatedRect(it->pos, it->extent) ;
		//cv::ellipse(image, rr, color) ;
	/*	
		cv::line(Idisplay, cv::Point2d(it->pos.at<float>(0), it->pos.at<float>(1)), 
				cv::Point2d(it->pos.at<float>(0) + it->extent.at<float>(0,0), it->pos.at<float>(1) + it->extent.at<float>(1,0)),
				cv::Scalar(0, 255, 0)) ;
				
		cv::line(Idisplay, cv::Point2d(it->pos.at<float>(0), it->pos.at<float>(1)), 
				cv::Point2d(it->pos.at<float>(0) + it->extent.at<float>(0,1), it->pos.at<float>(1) + it->extent.at<float>(1,1)),
				cv::Scalar(0, 255, 0)) ;
	*/	
	}	
}




