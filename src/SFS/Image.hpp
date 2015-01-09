#ifndef IMAGE
#define IMAGE


//#include "A3DA.hpp"
#include "ATDAException.hpp"
//#include <QtOpenGl>
//#define DEBUG
//#define TIMEM


//class Calibrator;
using namespace std;
//bool mycompare(cv::Rect, cv::Rect);


class Image {
	private:
		cv::Mat appleImg;														//obraz oryginalny
		cv::Mat appleGray;
		cv::Rect appleROI;
		vector<vector<cv::Point> > appleContour;
		int cols;
		int rows;
		cv::Point2i massCenter;
		cv::Mat appleMask;

		/**
		*	@fn znajduje srodek masy obiektu na zdjeciu
		*
		*/
		void findMassCenter(){
			cv::Moments mu = cv::moments(this->appleContour[0]);
			this->massCenter = cv::Point2i( (int)(mu.m10/mu.m00) , (int)(mu.m01/mu.m00) );
		}

		/**
		*	@fn szuka krawedzi obiektu na zdjeciu
		*
		*/
		void findEdge(){

			unsigned t0=clock();
			time_t start=time(0);				

			double largest_area=0;
			int largest_contour_index=0;
			double a;
			cv:: Rect bounding_rect;
 
			cv::Mat src = this->appleImg;
			cv::Mat thr(src.rows,src.cols,CV_8UC1); 
			cvtColor(src, thr, CV_BGR2GRAY);									//konwersja do skali szarosci
			threshold(thr, thr,25, 255,CV_THRESH_BINARY);						//progowanie
  
			vector<vector<cv::Point> > contours;									//miejsce na punkty konturow
			vector<cv::Vec4i> hierarchy;										//miejsce na hierarchie konturow
			findContours( thr, contours, hierarchy,CV_RETR_CCOMP, 
											CV_CHAIN_APPROX_SIMPLE );			//wyszukanie konturu na obrazie
			if(contours.size()<=0){
				throw ATDAException(NO_CONTOURS_FOUND,
						"Image::findEdge brak obiektu na zdjeciu");
			}
			for( unsigned int i = 0; i< contours.size(); i++ ){							//iteracja po wszystkich konturach
				a=contourArea( contours[i],false);								//sprawdzenie pola konturu
				if(a>largest_area){
					largest_area=a;
					largest_contour_index=i;									//zapamietanie indeksu najwiekszego konturu
					bounding_rect=boundingRect(contours[i]);					//wyszukanie roi dla najwiekszego konturu
				}
	
			}

			this->appleROI = bounding_rect;
			this->appleContour.push_back(contours[largest_contour_index]);

			time_t end=time(0);
			unsigned elapsed=clock()-t0;

#ifdef TIMEM
			cout << "Czas szukania konturu:" << endl;
			cout << elapsed<< " ms" <<  endl;
			cout << end-start << " s" <<  endl;
#endif

		}

		///wycinanie t³a (zastêpowanie czarnym) ----->TODO tworzenie maski
		void cropBackground(){
			this->appleMask = cv::Mat::zeros(this->rows, this->cols, CV_8UC1);
			unsigned t0=clock();
			time_t start=time(0);
			for(int i=0; i< this->rows; i++){
				for(int j=0; j< this->cols; j++){
					if ((cv::pointPolygonTest(this->appleContour[0], cv::Point(j,i), false))==-1  ||
						(cv::pointPolygonTest(this->appleContour[0], cv::Point(j,i), false))==0
						){
						this->appleGray.at<unsigned char>(i,j) = 0;
						
					}
					else{
						this->appleMask.at<unsigned char>(i,j) = 255;
					}
				}
			}
			time_t end=time(0);
			unsigned elapsed=clock()-t0;

#ifdef TIMEM
			cout << "Czas progowania tla:" << endl;
			cout << elapsed<< " ms" <<  endl;
			cout << end-start << " s" <<  endl;
#endif

		}


	public:

		/** 
		*	@fn konstruktor klasy przechowujacej obraz
		*	@param image macierz obrazu
		*	@param calibCrop zmienna decydujaca o uzyciu wycinania obrazu (w innym przypadku-> setMASK)
		**/
		Image(cv::Mat image, int calibCrop){
			try{
				this->appleImg = image;
				cv::cvtColor(this->appleImg, this->appleGray, CV_BGR2GRAY);

				this->cols = image.cols;
				this->rows = image.rows;
				this->findEdge();
				this->findMassCenter();
				if(calibCrop == 1){
					this->cropBackground();
				}
			}
			catch(cv::Exception &e){throw e;}
			catch(ATDAException &e){throw e;}
			catch(std::exception &e){throw e;}
		}


		cv::Point2i getMassCenter(){return this->massCenter;}
		///zwraca obraz oryginalny
		cv::Mat getAppleImg(){return this->appleImg;}
		///ilosc kolumn obrazu
		int getCols(){return this->cols;}
		///ilosc wierzy obrazu
		int getRows(){return this->rows;}
		///obraz w skali szarosci
		cv::Mat getAppleGray(){return this->appleGray;}
		cv::Rect getAppleROI(){return this->appleROI;}
		///kontur jablka
		vector<vector<cv::Point> > getAppleContour(){return this->appleContour;}
		cv::Mat getAppleMask(){return this->appleMask;}
		
		void setAppleContour(vector<vector<cv::Point> > contours){
			this->appleContour = contours;}
		void setAppleGray(cv::Mat gray){this->appleGray = gray;}
		void setAppleImg(cv::Mat img){this->appleImg = img;}
		void setAppleRoi(cv::Rect rect){this->appleROI = rect;}
		void setCols(int c){this->cols = c;}
		void setMassCenter(cv::Point2i point){this->massCenter = point;}
		void setRows(int r){this->rows = r;}
		void setAppleMask(cv::Mat mask){this->appleMask = mask;}

};

#endif
