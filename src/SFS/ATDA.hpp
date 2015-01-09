#ifndef ATDA
#define ATDA
/*** Apple Three-Dimensional Analysis**/
#define DEBUG
#define TIME_CHECK

#include <opencv/cv.h>
#include <opencv/ml.h>
#include <opencv2/core/core.hpp>                        //Sposób 
#include <opencv2/highgui/highgui.hpp>                    //dok³adny 
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>

//#include <QtOpenGl>
//#define DEBUG
//#define TIMEM

#include <vector>
#include <string> 
#include <iostream> 
#include <fstream>

//class Calibrator;
using namespace std;



/**
*	@struct struktura przechowujaca informacje o obiekcie ROI
*
*/
struct StatsRoi{

	cv::Rect rect;
	cv::Mat mask;
	int area;
	int areaRoi;
	int x;
	int y;
	//int nonZero;
	bool type;
	float depthAvg;
	float depthMax;
	float depthMin;
	float depthDev;

	float k4Avg;
	float k4Max;
	float k4Min;
	float k4Dev;

	float k6Avg;
	float k6Max;
	float k6Min;
	float k6Dev;

	float distX;
	float distY;
	
	cv::Mat response;
	cv::Mat selected;
};


struct Settings{
	int debugMode;
		//parametry algorytmow wizyjnych
		cv::Point3d light;
		int gaussMat;
		double gaussSig;
		int iter;
		int filterWindow;
		int backgroundCrop;
		double thresholdConcave;
		double thresholdStem;
		int thresholdArea;
	
		//parametry trybu uczenia i klasyfikatorow
		string fileSets;
		int numClasses;
		string fileRules;
		int storeMode;
		string fileClasses;
		string fileParams;
		string fileClassesCSV;
		string fileParamsCSV;
		const char *fileMLP;
		const wchar_t *path;

		int classifierMode;
		CvANN_MLP *mlp;
};



#endif
