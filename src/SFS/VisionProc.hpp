#ifndef VIS_PROC
#define VIS_PROC

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
#include "ATDA.hpp"
#include "Image.hpp"

class VisionProc{


	public:

		/**
		*	@fn porownanie dwóch roi po wspolrzednej x
		*	@param roi1 pierwszy prostokat
		*	@param roi2	drugi prostokat
		*	@return okreslenie czy pierwszy prostokat ma x mniejsze niz drugi
		**/
		static bool mycompare(StatsRoi roi1, StatsRoi roi2){
			if(roi1.rect.x > roi2.rect.x){
				return 0;
			}
			else{
				return 1;
			}

		}


		/**
		*	@fn znalezienie sciezki katalogu pliku przy podanej pelnej sciezce
		*	@param fullpath pelna sciezka pliku	
		*	@return katalog pliku
		*
		**/
		static string getPath(string fullpath){
			size_t end = fullpath.rfind('/');
			string path = fullpath.substr(0, end+1);
			return path;
		}


		/*
		*	@fn odczyt z pliku o podanej sciezce
		*	@param filename sciezka pliku obrazu
		*	@return obraz w formacie OpenCV
		*
		*/
		static cv::Mat readImage(std::string filename){
			cv::Mat img = cv::imread(filename);									///odczyt z pliku od podanej sciezce
			if(! img.data ){													///sprawdzenie czy plik istnieje
				throw  "Could not open or find the image" ;						///jesli nie - blad
			}
			return img;
		}

		/**
		*	@fn wycina nazwe pliku z pelnej sciezki
		*	@param fullpath pelna sciezka do pliku
		*	@return nazwa pliku
		*/
		static string fileName(string fullpath){
			size_t start = fullpath.rfind('/');
			size_t end = fullpath.rfind('.');
			string filename = fullpath.substr(start+1, end-start-1);
			return filename; 
		}


		/**
		*	@fn zamiana znakow w stringu
		*	@param subject pelny string
		*	@param search szukany znak
		*	@param replace znak na ktory zamieniamy
		*/
		static void ReplaceStringInPlace(std::string& subject,
					const std::string& search, const std::string& replace) {
			size_t pos = 0;
			while ((pos = subject.find(search, pos)) != std::string::npos) {
				 subject.replace(pos, search.length(), replace);
				pos += replace.length();
			}

		}


		/**
		*	@fn zapis macierzy do pliku xml
		*	@param mat macierz do zapisu
		*	@param filename nazwa pliku zapisu
		*	@param label etykieta danych do zapisu
		*
		**/
		static void fileStoreMat(cv::Mat mat, string filename, string label){
			cv::FileStorage tr(filename, cv::FileStorage::WRITE);
			tr << label << mat;
			tr.release();

		}


		/**
		*	@fn kowersja macierzy zapisanych w xml do pliku csv
		*	@param fileParams plik xml z parametrami probek
		*	@param fileClasses plik xml z klasami probek
		*	@param fileParamCSV	nazwa pliku z parametrami probek do zapisu w csv
		*	@param fileClassesCSV nazwa pliku z klasami probek do zapisu w csv
		*
		**/
		static void convertDataToCSV(string fileParams, string fileClasses,
									string fileParamCSV, string fileClassesCSV){

			cv::Mat training, classes;
			cv::FileStorage tr(fileParams, cv::FileStorage::READ);
			cv::FileStorage cl(fileClasses, cv::FileStorage::READ);
			tr["train"] >> training;											//TODO zamienic na label w parametrze funkcji
			cl["classes"] >> classes;
			tr.release();
			cl.release();

			fstream file;
			file.open(fileParamCSV.c_str(), ios::out);
			for(int y=0; y < training.rows; y++){
				for(int x=0; x < training.cols; x++){

					file << training.at<float>(y,x) << "\t";
				}
				file << endl;
			}

			file.close();

			file.open(fileClassesCSV.c_str(), ios::out);
			for(int y=0; y < classes.rows; y++){
				for(int x=0; x < classes.cols; x++){

					file << classes.at<float>(y,x) << "\t";
				}
				file << endl;
			}

			file.close();
		}



};



#endif
