#ifndef STEM_DETECTOR_HPP
#define STEM_DETECTOR_HPP

#include <opencv2/core/core.hpp>
#include <list>

#include "stem_descriptors.h"


class StemDetector {
public:
	cv::Mat inputImageHoles, inputImageStems, inputImageMask ; 

	//Scalespace parameters 
	float sigmaHoleStart;
	float sigmaHoleStop ;
	float sigmaHoleLogStep ;

	float sigmaStemStart ;
	float sigmaStemStop ;
	float sigmaStemLogStep  ;

	//Image processing paramters	
	int backgroundThreshold  ; //Threshold for separating backgroud from foreground
	float weakSuppRatio ; //All scalespace maxima responses smaller than the maximum response multiplied by this value are immediately ignored

	//Stem-hole selection parameters
	float blobRidgeResponseRatio ; //Blob responses are multiplied by this factor
	float holeStemResponseRatio ; //Hole responses are multiplied by this factor (accounts for possibly different hessian magnitudes in inputImageHoles and inputImageStems)
	float minHoleArea  ; //Minimum area of a hole
	float minStemAxisRatio  ; //Minimum ratio between ellipse axes lengths for the stem
	float maxStemAxisLength  ; //Maximum length of the short axis for the stem
	float minStemAxisLength ; //Minimum length of the long axis for the stem
	float maxStemHoleDist  ; //Maximum distance between (closer) stem end and the centre of the hole

	//Final decision settings
	float selectStemPairRatio  ; //Response for a pair is the sum of responses multiplied by this coefficient
	float reliableDetectionThreshold ; //Adjustable threshold of reliable detection

public:
	//Workspace data
	cv::Mat binaryMask ;
	
	std::vector<cv::Mat> Lsh ;
	std::vector<cv::Mat> Lss ;
	std::vector<cv::Mat> Rsh ;
	std::vector<cv::Mat> Rss ;
	std::vector<cv::Mat> Ixsh ;
	std::vector<cv::Mat> Iysh ;
	std::vector<cv::Mat> Ixss ;
	std::vector<cv::Mat> Iyss ;
	std::vector<float> sigmaHole, sigmaStem ;

	std::list<MaximumDescriptor> Lhlist ;
	std::list<MaximumDescriptor> Rhlist ;
	std::list<MaximumDescriptor> Rslist ;

	std::list<FeatureDescriptor> LhFlist ;
	std::list<FeatureDescriptor> RhFlist ;
	std::list<FeatureDescriptor> RsFlist ;

	std::list<FeatureDescriptor> hfList ;
	std::list<FeatureDescriptor> sfList ;

	std::list<FeatureDescriptor> holeList ;
	std::list<FeatureDescriptor> stemList ;
	std::list<FeatureDescriptor> pairList ;	
	std::list<FeatureDescriptor> holePairList ;
public:
	StemDetector()
	{
		// Scalespace parameters 
	this->sigmaHoleStart = 0.5f ;
	sigmaHoleStop = 30.0f ;
	sigmaHoleLogStep = 0.2f ;

	sigmaStemStart = 0.5f ;
	sigmaStemStop = 10.0f ;
	sigmaStemLogStep = 0.2f ;

	//Image processing paramters	
	backgroundThreshold = 15 ; //Threshold for separating backgroud from foreground
	weakSuppRatio = 0.5f ; //All scalespace maxima responses smaller than the maximum response multiplied by this value are immediately ignored

	//Stem-hole selection parameters
	blobRidgeResponseRatio = 0.4f ; //Blob responses are multiplied by this factor
	holeStemResponseRatio = 1.0f ; //Hole responses are multiplied by this factor (accounts for possibly different hessian magnitudes in inputImageHoles and inputImageStems)
	minHoleArea = 300.0f ; //Minimum area of a hole
	minStemAxisRatio = 2.0f ; //Minimum ratio between ellipse axes lengths for the stem
	maxStemAxisLength = 4.0f ; //Maximum length of the short axis for the stem
	minStemAxisLength = 5.0f ; //Minimum length of the long axis for the stem
	maxStemHoleDist = 30.0f ; //Maximum distance between (closer) stem end and the centre of the hole

	//Final decision settings
	selectStemPairRatio = 0.6f ; //Response for a pair is the sum of responses multiplied by this coefficient
	reliableDetectionThreshold = 0.11f ; //Adjustable threshold of reliable detection

	}

	~StemDetector() ;

	void computeScaleSpace(cv::Mat &inputImage, float sigma_begin, float sigma_end, float sigma_log_step, std::vector<float> &sigma, std::vector<cv::Mat> &Ls, std::vector<cv::Mat> &Rs, std::vector<cv::Mat> &Ixs, std::vector<cv::Mat> &Iys, bool computeBlobs) ; 
	void dilate3DArray(std::vector<cv::Mat> &Arr, std::vector<cv::Mat> &ArrRes, size_t ksize) ;
	void suppressNonMax(std::vector<cv::Mat> &Arr, std::vector<cv::Mat> &ArrMax) ;
	void computeObjectMask(int threshold, cv::Mat &mask) ;
	void filterWithBinaryMask(std::vector<cv::Mat> &Arr, cv::Mat &mask) ;
	void filterWeakMaxima(std::vector<cv::Mat> &Arr, double ratio) ;
	void extractMaximaList(std::vector<cv::Mat> &Arr, std::list<MaximumDescriptor> &descriptors) ;
	void describeBlobs(std::list<MaximumDescriptor> &mdescriptors, std::list<FeatureDescriptor> &fdescriptors, std::vector<float> &sigma) ;
	void describeValleys(std::list<MaximumDescriptor> &mdescriptors, std::vector<float> &sigma, std::vector<cv::Mat> &Ixs, std::vector<cv::Mat> &Iys, std::list<FeatureDescriptor> &fdescriptors) ;
	void describeValleys1(std::list<MaximumDescriptor> &mdescriptors, std::vector<float> &sigma, std::vector<cv::Mat> &Rss, std::list<FeatureDescriptor> &fdescriptors) ;
	static cv::RotatedRect getRotatedRect(const cv::Mat &pos, const cv::Mat &extent) ;
	bool isLarge(const FeatureDescriptor &fd) ;
	bool isOblong(const FeatureDescriptor &fd) ;
	bool isStemHoleRelated(const FeatureDescriptor &hole, const FeatureDescriptor &stem) ;
	void findPairs(const std::list<FeatureDescriptor> &holes, const std::list<FeatureDescriptor> &stems, std::list<FeatureDescriptor> &pairs) ;
	void findStems(const std::list<FeatureDescriptor> &features, std::list<FeatureDescriptor> &stems) ;
	void findHoles(const std::list<FeatureDescriptor> &features, std::list<FeatureDescriptor> &holes) ;

public:
	void setInputImages(cv::Mat &inputImageHoles, cv::Mat &inputImageStems, cv::Mat &inputImageMask) ;
	int detectHoleStem(FeatureDescriptor &selFeature) ;

	static void printMaximumDescriptors(std::list<MaximumDescriptor> &mdescriptors, std::vector<float> &sigma) ;
	static void printFeatureDescriptors(std::list<FeatureDescriptor> &fdescriptors) ;

	static void drawFeature(cv::Mat &image, const FeatureDescriptor &fdesc, cv::Scalar color = cv::Scalar(0, 255, 0)) ;
	static void drawFeatures(cv::Mat &image, std::list<FeatureDescriptor> &fdescriptors, cv::Scalar color = cv::Scalar(0, 255, 0)) ;
	static void displayImage(cv::Mat &image, const std::string &windowName="Display window") ;
	static void displayImages(std::vector<cv::Mat> &imspace) ;
} ;

#endif
