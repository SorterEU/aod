#ifndef ANALIZER
#define ANALIZER

#define _USE_MATH_DEFINES
#include "Image.hpp"
#include <time.h>
#include <ctime>
#include <algorithm> 
#include <math.h>
#include <cmath>
#include <complex>
#include "VisionProc.hpp"
#include <vector>
#include "StatsResolver.hpp"
//#include "Calibrator.hpp"
using namespace std;


class Analizer{
		public:

		/**
		*	@fn kontruktor klasy analizujacej w 3d obraz 
		*
		**/
		Analizer(Image *img, int gaussMat, double gaussSig, cv::Point3d light,
							int iter, int areaThreshold, 
							double thresholdConcave, double thresholdStem){
			// Pomiar czasu
			double t0, elapsed;
			time_t start, end;
			//
			try{
				// pocz¹tek zliczania czasu
				t0=clock();	
			    start=time(0);
				// 10x
				//for (int tind=0; tind<10; tind++) {

				this->depthMap = cv::Mat::zeros(img->getRows(), img->getCols(), 
							CV_64FC1);
				this->concave = cv::Mat::zeros(img->getRows(), img->getCols(), 
								CV_8UC1);
				this->stem = cv::Mat::zeros(img->getRows(), img->getCols(), 
								CV_8UC1);
				this->colorMap = cv::Mat::zeros(img->getRows(), img->getCols(), 
								CV_8UC3);
				for(int h = 0; h<6; h++){
					this->k[h] = cv::Mat::zeros(img->getRows(), img->getCols(),  
								CV_64FC1);
				}
				calcSFS(img, iter, gaussMat, gaussSig, light);
				//}
				// koniec pomiaru sfs
				end=time(0);
				elapsed= (double)(clock() - t0);
				cout << "Czas wykonania sfs 3D (10):" << endl;
				cout << elapsed << " ms" <<  endl;
				cout << end-start << " s" <<  endl;

				// pocz¹tek zliczania czasu
				t0=clock();	
			    start=time(0);
				// 10x
				//for (int tind=0; tind<10; tind++) {
				approximate(img, thresholdConcave, thresholdStem);
				//}
				// koniec pomiaru aproksymacji
				end=time(0);
				elapsed= (double)(clock() - t0) ;
				cout << "Czas wykonania aproksymacji 3D (11):" << endl;
				cout << elapsed << " ms" <<  endl;
				cout << end-start << " s" <<  endl;

				// pocz¹tek zliczania czasu
				t0=clock();	
			    start=time(0);
				// 10x
				//for (int tind=0; tind<10; tind++) {
				findROIs(areaThreshold);
				prepareStats(img);
				//}
				// koniec pomiaru
				end=time(0);
				elapsed= (double)(clock() - t0) ;

				cout << "Czas wykonania detekcji roi i cech (12a):" << endl;
				cout << elapsed << " ms" <<  endl;
				cout << end-start << " s" <<  endl;

			}
			catch( cv::Exception &e){throw e;}
			catch( ATDAException &e){throw e;}
			catch (std::exception &e){throw e;}
		} // Koniec konstruktora Analizer()

		//
		// Definicje funkcji dostêpu getxxx()
		cv::Mat getApproxMap() { return this->approxMap; }
		cv::Mat getColorMap(){ return this->colorMap; }
		cv::Mat getConcave(){ return this->concave; }
		cv::Mat getDepthMap(){	return this->depthMap; }
		cv::Mat* getK(){ return this->k; }
		vector<cv::Rect> getRois(){ return this->rois; }
		vector<cv::Rect> getRoisConc(){ return this->roisConc_; }
		vector<cv::Rect> getRoisStem(){ return this->roisStem_; }
		vector<StatsRoi> getStats(){ return this->stats; }
		vector<StatsRoi> getStatsConc(){ return this->statsConc_; }
		StatsResolver *getStatsResolver() { return this->statsRes; }
		vector<StatsRoi> getStatsStem(){ return this->statsStem_; }
		cv::Mat getStem(){ return this->stem; }
		
		//
		// Definicje funkcji zapisu do pól setxxx()
		void setApproxMap(cv::Mat map){this->approxMap = map;}
		void setColorMap(cv::Mat map){this->colorMap = map;}
		void setConcave(cv::Mat map){this->concave = map;}
		void setDepthMap(cv::Mat map){this->depthMap = map;}
		//void setK(cv::Mat *factors){this->k = factors;}
		void setRois(vector<cv::Rect> roiVec){this->rois = roiVec;}
		void setRoisConc(vector<cv::Rect> rois){this->roisConc_ = rois;}
		void setRoisStem(vector<cv::Rect> rois){this->roisStem_ = rois;}
		void setStats(vector<StatsRoi> statsVec){this->stats = statsVec;}
		void setStatsConc(vector<StatsRoi> stats){this->statsConc_ = stats;}
		void setStatsRes(StatsResolver *resolver){this->statsRes = resolver;}
		void setStatsStem(vector<StatsRoi> stats){this->statsStem_ = stats;}
		void setStem(cv::Mat stemMat){this->stem = stemMat;}

		// Struktura obiektu klasy Analizer
	private:
		cv::Mat depthMap;
		cv::Mat approxMap;
		cv::Mat k[6];
		cv::Mat colorMap;
		StatsResolver *statsRes;
		cv::Mat concave;
		cv::Mat stem;
		vector<cv::Rect> rois, roisStem_, roisConc_;
		vector<StatsRoi> stats, statsStem_, statsConc_;
		

		// Defincije funkcji prywatnych klasy
		/**
		*	@fn realizuje algorytm SFS metoda przyblizenia pochodnych czastkowych
		*		w plaszczyznach x i y
		*	@param img klasa przechowujaca obraz
		*	@param iter liczba iteracji algorytmu SFS
		*	@param gaussMat wymiar macierzy filtru Gaussa
		*	@param gaussSig sigma filtru Gaussa
		*	@param light wspolrzedne zrodla swiatla
		*/
		void calcSFS(Image *img, int iter, int gaussMat, double gaussSig, 
										cv::Point3d light){
			
			unsigned t0=clock();
			time_t start=time(0);
			try{									
				double p = 0.0;		double q = 0.0;			//inicjalizacja wymaganych zmiennych
				int szy = img->getRows();
				int szx = img->getCols();
				double Wn = 0.001*0.001;
				double ps = light.x / light.z;
				double qs = light.y / light.z;
				double pq = 0.0;
				double fZ = 0.0;
				double dfZ = 0.0;
				double y, k, d;
				double pqs = 1 + ps * ps + qs * qs;
				double sqrtpqs = sqrt(pqs);
		
				cv::Mat zn = cv::Mat::zeros(szy, szx, CV_64FC1);
				cv::Mat zn1 = cv::Mat::zeros(szy, szx, CV_64FC1);
				cv::Mat si1 = cv::Mat::ones(szy, szx, CV_64FC1);
				cv::Mat si = cv::Mat::zeros(szy, szx, CV_64FC1);
				
				cv::Mat tmp = img->getAppleGray().clone();
				
				cv::GaussianBlur(tmp, tmp, cv::Size(gaussMat, gaussMat), 
										gaussSig, cv::BORDER_DEFAULT);		//filtr Gaussa - redukcja szumu
				
				tmp.convertTo(tmp, CV_64FC1);									//konwersja do doubl
				cv::minMaxLoc(tmp, NULL, &d, NULL, NULL);
				//obliczanie mapy w kolejnych iteracjach
				double zn1atij, sqrtpq, si1atij, sqrtpqsqrtpqs, ppsqqs;
				for (int l=0; l< iter; l++){
					for (int i = 0; i<szy; i++){
						for (int j=0; j<szx;j++){
							zn1atij = zn1.at<double>(i, j);
							if (j < 2 || i  < 2 || j == szx || i == szy){
								p = 0.0;  q = 0.0;
							} else {	
								p = zn1atij - zn1.at<double>(i, j - 1);	// przyblizenie dyskretne pochodnych czastkowych
								q = zn1atij - zn1.at<double>(i - 1, j);
							}
							pq = 1 + p * p + q * q;
							sqrtpq = sqrt(pq);
							tmp.at<double>(i,j) = tmp.at<double>(i,j) / d;
							sqrtpqsqrtpqs = sqrtpq * sqrtpqs; 
							ppsqqs = 1.0 + p * ps + q * qs;

							fZ = -1.0 * (tmp.at<double>(i,j) - max(0.0, (ppsqqs/ sqrtpqsqrtpqs)) );
							dfZ = -1.0* (max(1.0, ((ps + qs) / sqrtpqsqrtpqs ) - ( (p + q) * ppsqqs / 
								(sqrtpq * pq * sqrtpqs) )));

							y = fZ + dfZ * zn1atij;
							si1atij = si1.at<double>(i, j);
							k = si1atij * dfZ / (Wn + dfZ * si1atij * dfZ);
							si.at<double>(i,j) = (1 - k * dfZ) * si1atij;
							zn.at<double>(i,j) = zn1atij + 	k * (y - dfZ * zn1atij);

						}
					}
					zn1 = zn.clone();
					si1 = si.clone();
				}
				this->depthMap = zn1.clone();
			}
			catch(cv::Exception &e){throw e;}

			time_t end=time(0);
			unsigned elapsed=clock()-t0;

#ifdef TIMEM
			cout << "Czas wykonania SFS:" << endl;
			cout << elapsed<< " ms" <<  endl;
			cout << end-start << " s" <<  endl;
#endif			
		}



		/**
		*	@fn oblicza wspolczynniki funkcji kwadratowej dwu zmienych dla 
		*		podanej mapy glebokosci
		*	@param img obraz
		*	@e ------wielkosc sasiedztwa deprecated
		*	
		*
		*/
		void approximate(Image *img, double threshold, double stemThreshold){
#ifdef TIMEM
			unsigned t0=clock();
			time_t start=time(0);
#endif
			int e=2;
		
			int rows = img->getRows();
			int cols = img->getCols();

			cv::Mat zMap; 
			cv::Mat helpMap = cv::Mat::zeros(rows, cols, CV_64FC1);
			double top[6];
			double bot[6];
			double zatwk;
			this->depthMap.copyTo(zMap);
			int r,c,w,k;
			try{
				for(r=0; r< rows; r++){
					for(c=0; c< cols; c++){
						for(int h = 0; h<6; h++){
							top[h] = 0.0;											//zerowanie zmiennych pomocniczych
							bot[h] = 0.0;
						}
						if (c>e && r>e && r< rows-e && c < cols - e){	//pomijanie obszarow brzegowych
							for(w = r-e; w <= r+e; w++ ){							//badanie sasiednich pikseli
								for(k = c-e; k <= c+e; k++){
									zatwk = zMap.at<double>(w,k);
									top[0] += zatwk ; 		//zastosowanie wzorow na wspolczynniki funkcji
									bot[0] += 1;
									top[1] += (k-c)*zatwk;
									bot[1] += (k-c)*(k-c);
									top[2] += (w-r)*zatwk;
									bot[2] += (w-r)*(w-r);
									top[3] += (((k-c)*(k-c))-2)*zatwk ;
									bot[3] += (((k-c)*(k-c))-2)*(((k-c)*(k-c))-2);
									top[4] += ((k-c)*(w-r))*zatwk ;
									bot[4] += ((k-c)*(w-r))*((k-c)*(w-r));
									top[5] += (((w-r)*(w-r))-2)*zatwk ;
									bot[5] += (((w-r)*(w-r))-2)*(((w-r)*(w-r))-2);

								}
							}
							for(int h = 0; h<6; h++){
								this->k[h].at<double>(r,c) = top[h]/bot[h];
							}
						
						}
						helpMap.at<double>(r,c) = this->k[0].at<double>(r,c) +
							this->k[1].at<double>(r,c) + this->k[2].at<double>(r,c)+
							this->k[3].at<double>(r,c) + this->k[4].at<double>(r,c)+
							this->k[5].at<double>(r,c);
						if(img->getAppleMask().at<unsigned char>(r,c) != 0){			//progowanie mapy
							this->makeMaps(img, r,c, threshold, stemThreshold );		//progi zadane dla odcinania szumu TODO :sterowalne
						}
					}
				}
			}
			catch (cv::Exception &e){
				throw e;
			}
			catch (std::exception &e){
				throw e;
			}

			this->approxMap = helpMap;

#ifdef TIMEM
			time_t end=time(0);
			unsigned elapsed=clock()-t0;

			cout << "Czas wykonania aproksymacji i mapowania:" << endl;
			cout << elapsed<< " ms" <<  endl;
			cout << end-start << " s" <<  endl;
#endif
		}

		
		/**
		*	@fn realizuje zadanie progowania mapy w celu eliminacji szumu 
				oraz wykrycia defektow/zaglebien/szypulek
		*	@param img obraz
		*	@param r wiersz (row) badanego punktu rownowazne ze wspolrzedna y obrazu
		*	@param c kolumna badanego punktu 
		*	@param threshold prog wkleslych obszarow - odrzucenie szumu
		*	@param stemThreshold prog ogonkow
		*/
		void makeMaps(Image* img, int r, int c, double threshold, 
						double stemThreshold){

			if((this->k[3].at<double>(r,c) > threshold) &&						//szukanie wkleslosci
				(this->k[5].at<double>(r,c) > threshold)) {
				colorMap.at<cv::Vec3b>(r,c)[2] =  255; 
				this->concave.at<unsigned char>(r,c) = 255;
			}
			else if((this->k[3].at<double>(r,c) < threshold) &&					//szukanie wypuklosci
				(this->k[5].at<double>(r,c) < threshold)){
				colorMap.at<cv::Vec3b>(r,c)[1] =  255; 	
			}
			else if((this->k[3].at<double>(r,c) < threshold) &&					//szukanie oszarow siodlowych
				(this->k[5].at<double>(r,c) > threshold)){
				colorMap.at<cv::Vec3b>(r,c)[0] =  255; 
				this->concave.at<unsigned char>(r,c) = 255;		
			}
			else{
				this->concave.at<unsigned char>(r,c) = 255;	
			}

			if((this->k[3].at<double>(r,c) < stemThreshold) ||					//szukanie obszarow brzegowych 
				(this->k[5].at<double>(r,c) < stemThreshold)){					//w tym szypulek poza obrebem jablka
				colorMap.at<cv::Vec3b>(r,c)[2] =  134;
				colorMap.at<cv::Vec3b>(r,c)[1] =  3;
				colorMap.at<cv::Vec3b>(r,c)[0] =  199;
				this->stem.at<unsigned char>(r,c) = 255;
			}
		}


		/**
		*	@fn przygotowanie statystyk dla kazdego znalezionego roi
		*	@param	img obraz
		*
		**/
		void prepareStats(Image *img){
			try{
				unsigned t0=clock();
				time_t start=time(0);

				this->statsRes = new StatsResolver(this->stem, this->concave, 
									this->k[3], this->k[5], this->approxMap, 
									img->getAppleGray());
				for(unsigned int i=0; i<this->roisConc_.size(); i++){	
					this->statsRes->calcStats(img, roisConc_[i], 0); 
					this->statsConc_.push_back(this->statsRes->getStats());
				}
				for(unsigned int i=0; i<this->roisStem_.size(); i++){
					this->statsRes->calcStats(img, roisStem_[i], 1); 
					this->statsStem_.push_back(this->statsRes->getStats());
				}
				delete statsRes;

				this->stats.reserve(this->statsStem_.size() + 
										this->statsConc_.size());
				this->stats.insert(this->stats.end(), this->statsConc_.begin(), 
										this->statsConc_.end());
				this->stats.insert(this->stats.end(), this->statsStem_.begin(), 
										this->statsStem_.end());
				//sort(this->stats.begin(), this->stats.end(), 
				//						&VisionProc::mycompare); //sortowanie roi po zmiennej x

				time_t end=time(0);
				unsigned elapsed=clock()-t0;

#ifdef TIMEM
				cout << "Czas liczenia statystyk:" << endl;
				cout << elapsed<< " ms" <<  endl;
				cout << end-start << " s" <<  endl;
#endif
			}
			catch(ATDAException &e){throw e;}
			catch(cv::Exception &e){throw e;}
			catch(std::exception &e){throw e;}
		}

		/**
		*	@fn dodaje znalezione roi do odpowiedniego wektora filtrujac je 
			po wielkosci
		*	@param contours kontury defektow/zaglebnien
		*/
		void addRoi(vector<vector<cv::Point> > contours, bool type, 
												int areaThreshold ){
			cv::Rect bounding_rect;
			for( unsigned int i = 0; i< contours.size(); i++ ){					//iteracja po kazdym konturze
				if (contourArea(contours[i], false) > areaThreshold ){			//TODO: zaleznie od settings \(wielkosci)
					bounding_rect=cv::boundingRect(contours[i]);
					if(type==0){
						this->roisConc_.push_back(bounding_rect);
					}
					else{
						this->roisStem_.push_back(bounding_rect);
					}
					
				}
			}

		}


		/** 
		*	@fn odszukanie roi za pomoca wyznaczania konturu na mapie BW
		*	@param areaThreshold okresla jak duze defekty przyjmujemy do analizy
		*
		**/
		void findROIs(int areaThreshold){
			unsigned t0=clock();
			time_t start=time(0);
			cv::Mat src = this->concave.clone();
			cv::Mat src2 = this->stem.clone();
			vector<vector<cv::Point> > contours, contours2;						//miejsce na kontury
			vector<cv::Vec4i> hierarchy, hierarchy2;
			cv::Rect bounding_rect;
			findContours( src, contours, hierarchy, CV_RETR_CCOMP, 
										CV_CHAIN_APPROX_SIMPLE );				//znalezienie konturow 
			findContours( src2, contours2, hierarchy2, CV_RETR_CCOMP,
										CV_CHAIN_APPROX_SIMPLE );				//w obu typach macierzy
			if (contours.size() <=0 || contours2.size()<=0){
				throw ATDAException(NO_CONTOURS_FOUND,
					"Analizer::findROIs brak konturow zaglebien");
			}
			this->addRoi(contours, 0, areaThreshold);
			this->addRoi(contours2, 1, areaThreshold);
			time_t end=time(0);
			unsigned elapsed=clock()-t0;
#ifdef TIMEM
			cout << "Czas szukania ROI:" << endl;
			cout << elapsed<< " ms" <<  endl;
			cout << end-start << " s" <<  endl;
#endif
		}
		



}; // Koniec definicji klasy Analizer




#endif

