#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include "stem_detector.hpp"
#include "tic_toc.hpp"
//#include "dirent.h"

//namespace STEM {

// Function stem 1

void displayImageWithFeatures(cv::Mat &image, cv::Mat &Idisplay, std::list<FeatureDescriptor> &fdescriptors, 
							  const std::string &windowName="Display window", 
							  const cv::Scalar &color = cv::Scalar(0, 255, 0)) 
{ 
	cv::namedWindow( windowName, cv::WINDOW_AUTOSIZE );
	Idisplay = image.clone() ;	

	StemDetector::drawFeatures(Idisplay, fdescriptors, color) ;	

	if (!fdescriptors.empty()) 
		StemDetector::drawFeature(Idisplay, fdescriptors.back(), cv::Scalar(0,0,255)) ; //Just for testing
	
	cv::imshow( windowName, Idisplay);                   
	
}

// Function stem 2

void initRedAppleRGB(StemDetector &sd, std::list<std::string> &fileNames, cv::Rect &roi)
{
	sd.backgroundThreshold = 15 ;
	sd.reliableDetectionThreshold = 0.07f ;
	roi = cv::Rect(450, 390, 130, 130) ; //RGB - czerwone 
	for(int i = 2; i <= 40 ; i = i + 2) 
		fileNames.push_back(std::string("./apples/") + std::to_string(i) + ".bmp") ;
}

// Function stem 3

void static initRedYellowAppleRGB(StemDetector &sd, std::list<std::string> &fileNames, cv::Rect &roi)
{
	sd.backgroundThreshold = 15 ;
	sd.reliableDetectionThreshold = 0.17f ;
	roi = cv::Rect(430, 610, 130, 130) ; //RGB - czerwone 
	for(int i = 2; i <= 40 ; i = i + 2) 
		fileNames.push_back(std::string("./apples/") + std::to_string(i) + ".bmp") ;
}


//void generateFileList(const std::string &dirName, const std::string &mask, std::list<std::string> &fileNames)
//{
//	DIR *dir;
//	struct dirent *ent;
//	fileNames.clear() ;
//	if ((dir = opendir(dirName.c_str())) != NULL) {
//		while ((ent = readdir (dir)) != NULL) {
//			std::string fileName = ent->d_name ;
//			if (fileName.find(mask) != std::string::npos)
//				fileNames.push_back(dirName + "/" + fileName) ;
//		}
//		closedir (dir);
//	}
//	fileNames.sort() ;
//}


// Function stem 4

void initApple1(StemDetector &sd, std::list<std::string> &fileNames)
{
	//generateFileList("/home/artur/sorter/Database/01_Diffuse/01/A", "Apple1", fileNames) ;
	//sd.reliableDetectionThreshold = 0.07 ;
	//fileNames.clear() ;
	//fileNames.push_back("/home/artur/sorter/Database/01_Diffuse/01/A/2013-10-31T15_50_08.819622_Apple1.png") ;
	//fileNames.push_back("/home/artur/sorter/Database/01_Diffuse/01/A/2013-10-31T15_50_06.334922_Apple1.png") ;
	sd.backgroundThreshold = 20 ;
	sd.selectStemPairRatio = 0.8f ;
}

// Function stem 5

void preprocessImage1(const cv::Mat &inputImage, StemDetector &sd)
{
	cv::Mat planes[3];
	cv::split(inputImage, planes);

	cv::Mat imageGrey ; 	 
	cv::cvtColor(inputImage, imageGrey, CV_BGR2GRAY);

	cv::Mat inputImageHoles = imageGrey * 0.3 + planes[2] - planes[1] - planes[0] ;
	cv::Mat inputImageStems = planes[2] ;
	cv::Mat inputImageMask = planes[2] ;
	
	sd.setInputImages(inputImageHoles, inputImageStems, inputImageMask) ;
	
	/*
	double minVal, maxVal ;
	cv::minMaxIdx(outputImage, &minVal, &maxVal) ;
	std::cout << "MinMax " << minVal << " " << maxVal << std::endl ;
	*/
}


// Function stem 6

void initApple2(StemDetector &sd, std::list<std::string> &fileNames)
{
	//generateFileList("/home/artur/sorter/Database/01_Diffuse/02/B", "Apple1", fileNames) ;
	sd.reliableDetectionThreshold = 0.11 ;
	sd.backgroundThreshold = 20 ;
	//sd.blobRidgeResponseRatio = 0.3f ; 	
}

// Function stem 7

void preprocessImage2(const cv::Mat &inputImage, StemDetector &sd)
{
	cv::Mat planes[3];
	cv::split(inputImage, planes);

	cv::Mat imageGrey ; 	 
	cv::cvtColor(inputImage, imageGrey, CV_BGR2GRAY);

	cv::Mat inputImageHoles = imageGrey ;
	cv::Mat inputImageStems = imageGrey ;
	cv::Mat inputImageMask = imageGrey ;
	
	sd.setInputImages(inputImageHoles, inputImageStems, inputImageMask) ;
}

//void initApple3(StemDetector &sd, std::list<std::string> &fileNames)
//{
//	// generateFileList("/home/artur/sorter/Database/01_Diffuse/03/B", "Apple1", fileNames) ;
//
//	sd.reliableDetectionThreshold = 0.11 ;
//	sd.backgroundThreshold = 20 ;
//}


// Function stem 8 

void preprocessImage3(const cv::Mat &inputImage, StemDetector &sd)
{
	cv::Mat planes[3];
	cv::split(inputImage, planes);

	cv::Mat imageGrey ; 	 
	cv::cvtColor(inputImage, imageGrey, CV_BGR2GRAY);

	cv::Mat inputImageHoles = imageGrey ;
	cv::Mat inputImageStems = imageGrey ;
	cv::Mat inputImageMask = imageGrey ;
	
	sd.setInputImages(inputImageHoles, inputImageStems, inputImageMask) ;
}


//////////////////////////
// Pierwsza g³ówna funkcja w STEM

FeatureDescriptor SzypulkaWObrysie(char* ouDir, char* ouName, int VISUALsw, int LOGSsw, cv::Mat imageRGB, 
					 float relDetThresh= 0.11f, int backThresh = 20, float selStemPR = 0.8f) // original AW: main()
{

	cv::Rect roi ;
	StemDetector sd ;
	cv::Mat image, retImage;
	char outName[120];

	FeatureDescriptor selFeature; // dla zwracanego wyniku

	//std::list<std::string> fileNames ;
	//initApple3(sd, fileNames) ;

	// Ustaw kluczowe parametry analizy obrazu
	sd.reliableDetectionThreshold = relDetThresh; // 0.11 ;
	sd.backgroundThreshold = backThresh; // 20 ;
	sd.selectStemPairRatio = selStemPR; // 0.8f ;

//	for (std::list<std::string>::iterator it = fileNames.begin(); it != fileNames.end() ; it++) {
//		std::cout << "Filename " << *it << std::endl ;
	
	image = imageRGB; 
// image = cv::imread(*it, CV_LOAD_IMAGE_COLOR);   // Read the file

	if(! image.data )  // Check for invalid input
	{
		std::cout <<  "no data " << std::endl;
		selFeature.decision = -1;
		return selFeature;
	}

// Funkcja 1
	// Ustaw trzy obrazy
	preprocessImage3(image, sd) ;

// Funkcja 2 - najwa¿niejsza
		// Funkcja sd.detectHoleStem(selFeature) wywo³uje g³ówn¹ funkcjê obliczeniow¹ klasy StemDetector.
		// Wynik: decision = 0 - brak detekcji zag³êbienia/szypu³ki
		//		decision = 1 - detekcja zag³êbienia/szypu³ki
		// Porównuje ocenê najlepszego wykrytego obiektu, z ustalonym progiem: StemDetector::reliableDetectionThreshold
		// W przypadku detekcji wype³niana jest te¿ zmienna selFeature (klasy FeatureDescriptor) - opisuj¹ca wykryty obiekt
		// (za poœrednictwem elips), tzn.
		// 1. Mo¿liwa jest jedna lub dwie elipsy (zag³êbienie lub zag³êbienie/szypu³ka)
		// 2. selFeature.pos - macierz 2x1  lub 2x2 - której kolumny okreœlaj¹ wspó³rzêdne œrodków elips
		// 3. selFeature.extent  - macierz 2x2 lub 2x4 - której kolumny okreœlaj¹ wektory krótkiej i d³ugiej osi elipsy
		// 4. selFeature.response - ocena obiektu
		// Np. pos = [206, 213;  150, 131]
		// extent = [21.18783, 0, 1.9659238, -3.1171303;
		//	      0, 21.18783, 0.49629286, 12.347631]
		// response=0.235003
		// Mamy tutaj dwie elipsy: zag³êbienie i szypu³kê:
		// - Œrodek pierwszej elipsy: [260,150]', Œrodek drugiej elipsy: [213,131]'
		// - Wektor osi krótkiej pierwszej elipsy [21.18783, 0]'
		// - Wektor osi d³ugiej pierwszej elipsy [0, 21.18783]'
		// - Wektor osi krótkiej drugiej elipsy [1.9659238, 0.49629286]'
		// - Wektor osi d³ugiej drugiej elipsy [-3.1171303, 12.347631]'
		
	
	int decision = sd.detectHoleStem(selFeature) ;

	selFeature.decision = decision;

// Wizualizacja wyników
	if (VISUALsw == 1) {
		std::cout << "Holes:" << std::endl ;
		sd.printFeatureDescriptors(sd.holeList) ;
		std::cout << "Stems:" << std::endl ;
		sd.printFeatureDescriptors(sd.stemList) ;
		std::cout << "Pairs:" << std::endl ;
		sd.printFeatureDescriptors(sd.pairList) ;

		std::cout << "Decision = " << decision << std::endl ;
		std::list<FeatureDescriptor> selList ;
			
		if (decision == 1) {
			std::cout << "Sel. feature pos = " << selFeature.pos << std::endl ;
			std::cout << "Sel. feature extent = " << selFeature.extent << std::endl ;
			std::cout << "Sel. feature response = " << selFeature.response << std::endl ;
			selList.push_back(selFeature) ;
		} 
		else 
		{
			if (!sd.holePairList.empty()) {
				std::cout << "Best feature pos = " << sd.holePairList.back().pos << std::endl ;
				std::cout << "Best feature extent = " << sd.holePairList.back().extent << std::endl ;
				std::cout << "Best feature response = " << sd.holePairList.back().response << std::endl ;
			}
		}

		StemDetector::displayImage(sd.binaryMask, "Binary mask") ;
		StemDetector::displayImage(sd.inputImageHoles, "Input holes") ;
		StemDetector::displayImage(sd.inputImageStems, "Input stems") ;

		displayImageWithFeatures(image, retImage, sd.RhFlist, "Hole ridges") ;
		displayImageWithFeatures(image, retImage, sd.LhFlist, "Hole blobs") ;
		displayImageWithFeatures(image, retImage, sd.RsFlist, "Stem ridges") ;
		displayImageWithFeatures(image, retImage, sd.holeList, "Holes") ;
		if (LOGSsw == 1) {
			// Write out the image
			sprintf(outName, "%s/2D_2_Holes_%s.png", ouDir, ouName);
		        	cv::imwrite(outName, retImage);

		}
		displayImageWithFeatures(image, retImage, sd.stemList, "Stems") ;
		if (LOGSsw == 1) {
			// Write out the image
			sprintf(outName, "%s/2D_3_Stems_%s.png", ouDir, ouName);
		        	cv::imwrite(outName, retImage);

		}
		displayImageWithFeatures(image, retImage, sd.pairList, "Pairs") ;
		if (LOGSsw == 1) {
			// Write out the image
			sprintf(outName, "%s/2D_4_Pairs_%s.png", ouDir, ouName);
		        	cv::imwrite(outName, retImage);
		}
		displayImageWithFeatures(image, retImage, selList, "Selected feature") ;

		if (LOGSsw == 1) {
			// Write out the image
				sprintf(outName, "%s/2D_5_Selected_%s.png", ouDir, ouName);
		        	cv::imwrite(outName, retImage);

		}

		retImage.release();
		image.release();
	}


		//cv::waitKey(0) ;

		//StemDetector::displayImages(sd.Rss) ;

	return selFeature;
	}

//////////////////////////////////
// Druga g³ówna funkcja w STEM:
	FeatureDescriptor OdstajacaSzypulka(char* fileName, int visual, int logs, cv::Mat imageRGB)
	{
		// TO DO
		FeatureDescriptor selFeature;
		// int decision = 0;

		selFeature.decision = 0;
		return selFeature;
	}

//}
// end namespace STEM
