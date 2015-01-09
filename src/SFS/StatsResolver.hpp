#ifndef STATS
#define STATS

//#include "A3DA.hpp"
#include "Image.hpp"


class StatsResolver{

	private:
		StatsRoi stats_;
		cv::Mat cut_;
		cv::Mat stem_, concave_, k4_, k6_, depth_;
		cv::Mat gray_;

		/**
		*	@fn wycina ROi z calego obrazu
		*	@param src oryginalny obraz
		*	@param roi obszar do wyciecia
		*/
		void cutMat(cv::Mat src, cv::Rect roi){
			if(roi.area()==0){
				throw ATDAException(EMPTY_METHOD_PARAM, 
					"StatsResolver::cutMat roi");
			}
			else if(!src.data){
				throw ATDAException(EMPTY_METHOD_PARAM,
					"StatsResolver::cutMat src");
			}
			else{
				this->cut_ = src(roi).clone(); 
			}
		}


		/**
		*	@fn podaje pole roi (prostokata)
		*
		*/
		int findAreaRoi(){
			return this->cut_.cols * this->cut_.rows;
		}


		/**
		*	@fn podaje pole samej zmiany ksztaltu
		*	@param roi obszar
		*/
		int findArea(cv::Rect roi){
			cv::Mat dst;
			cv::add(this->concave_, this->stem_, dst);
			//imshow("dst", dst);
			return cv::countNonZero(dst(roi));

		}


		/**
		*	@fn liczy niezerowe piksele w roi --deprecated--
		*
		*/
		int findNonZero(cv::Rect roi){
			return cv::countNonZero(this->gray_(roi));
		}


		/**
		*	@fn oblicza srednia i odchylenie standardowe dla macierzy
		*	@param matrix macierz
		*	@param m srednia
		*	@param d odchylenie standardowe
		*/
		void findMeanDev(cv::Mat matrix, float *m, float *d){
			cv::Mat mean, dev;					// 0 - srednia, 1 -odchylenie
			if(!matrix.data){
				throw ATDAException(EMPTY_METHOD_PARAM,
					"StatsResolver::findMeanDev matrix");
			}
			cv::meanStdDev(matrix, mean, dev);
			*m = (float)mean.at<double>(0,0);
			*d = (float)dev.at<double>(0,0);
			
		}
		

		/**
		*	@fn oblicza minmialna i maksymalna wartosc macierzy
		*	@param matrix macierz
		*	@param min minimalna wartosc
		*	@param max maksymalna wartosc 
		*/
		void findMinMaxValue(cv::Mat matrix, float *min, float *max){
			double minMax[2];                   //0- min, 1 max
			if(!matrix.data){
				throw ATDAException(EMPTY_METHOD_PARAM,
					"StatsResolver::findMinMaxValue matrix");
			}
			cv::Point minLoc, maxLoc;

			cv::minMaxLoc(matrix, &minMax[0], &minMax[1], &minLoc, &maxLoc);
			*min = (float)minMax[0];
			*max = (float)minMax[1];
		
		}


		/**
		*	@fn oblicza odleglosc obszaru roi od srodka masy obiektu jablka
		*	@param massCenter srodek masy jablka
		*	@param roi	obszar zainteresowania
		*	@param xd zmienna x wektora odleglosci
		*	@param yd zmienna y wektora odleglosci
		*/
		void findDistXY(cv::Point2i massCenter, cv::Rect roi, float &xd,
									float &yd){
			if(roi.area() <=0){
				throw ATDAException(EMPTY_METHOD_PARAM,
					"StatsResolver::findDistXY matrix");
			}

			int centerX, centerY;

			centerX = (int)(roi.width/2) + roi.x;
			centerY = (int)(roi.height/2) + roi.y;
			
			xd = (float)(massCenter.x - centerX);///(float)roi.width;
			yd = (float)(massCenter.y - centerY);///(float)roi.height;
			
		}

		/**
		*	@fn wydziela maske dla danego obszaru zainteresowania
		*	@param roi obszar zainteresowania
		*	@param type typ zadanego obszaru (brzeg jablka/wkleslosc w srodku)
		*/
		cv::Mat makeMask(cv::Rect roi, int type){
			cv::Mat mask;
			if(roi.area() <=0){
				throw ATDAException(EMPTY_METHOD_PARAM,
					"StatsResolver::makeMask roi");
			}
			if(type = 0){
				mask = this->concave_(roi).clone();
			}
			else{
				mask = this->stem_(roi).clone();
			}
			return mask;
		}


	public:

		/**
		*	@fn zbiera wszystkie statystyki dla jednego roi
		*	@param img klasa obrazu
		*	@param roi obszar zainteresowania
		*	@param type typ obszaru (brzeg jablka/wkleslosc)
		*/
		void calcStats(Image *img, cv::Rect roi, bool type){
			try{
				this->stats_.rect = roi;
				cutMat(img->getAppleGray(), roi);
				this->stats_.areaRoi = findAreaRoi();
				this->stats_.area = findArea(roi);
				this->stats_.x = roi.width;
				this->stats_.y = roi.height;
				this->stats_.type = type;
				findMeanDev(this->depth_(roi), &this->stats_.depthAvg,
											&this->stats_.depthDev );
				findMinMaxValue(this->depth_(roi), &this->stats_.depthMin, 
											&this->stats_.depthMax);
				findDistXY(img->getMassCenter(), roi, this->stats_.distX, 
											this->stats_.distY);
				findMeanDev(this->k4_(roi), &this->stats_.k4Avg, 
											&this->stats_.k4Dev);
				findMinMaxValue(this->k4_(roi), &this->stats_.k4Min,
											&this->stats_.k4Max);
				findMeanDev(this->k6_(roi), &this->stats_.k6Avg, 
											&this->stats_.k6Dev);
				findMinMaxValue(this->k6_(roi), &this->stats_.k6Min, 
											&this->stats_.k6Max);

				this->stats_.mask = makeMask(roi, type);
			}
			catch(ATDAException &e){throw e;}
			catch(cv::Exception &e){throw e;}
			catch(std::exception &e){throw e;}
		}

		/**
		*	@fn konstruktor klasy odpowiedzialnej za statystyki roi
		*	@param stem macierz wkleslosci brzegowych jablka
		*	@param concave macierz wkleslosci jablka
		*	@param k4 macierz wspolczynnikow funkcji kwadratowej w plaszczyznie x
		*	@param k6 macierz wspolczynnikow funkcji kwadratowej w plaszczyznie y
		*	@param depth macierz glebokosci obiektu
		*	@param gray macierz obrazu w skali szarosci
		*/
		StatsResolver(cv::Mat stem, cv::Mat concave, cv::Mat k4,
						cv::Mat k6, cv::Mat depth, cv::Mat gray){
			if(!stem.data || !concave.data || !k4.data || !k6.data ||
				!depth.data || !gray.data){
					throw ATDAException(EMPTY_METHOD_PARAM,
					"StatsResolver::StatsResolver");;
			}
			else{
				this->stem_ = stem.clone();
				this->concave_ = concave.clone();
				this->k4_ = k4.clone();
				this->k6_ = k6.clone();
				this->depth_ = depth.clone();
				this->gray_ = gray.clone();
			}
		}

		cv::Mat getConcave(){return this->concave_;}
		cv::Mat getCut(){return this->cut_;}
		cv::Mat getDepth(){return this->depth_;}
		cv::Mat getGray(){return this->gray_;}
		cv::Mat getK4(){return this->k4_;}
		cv::Mat getK6(){return this->k6_;}
		StatsRoi getStats(){return this->stats_;}
		cv::Mat getStem(){return this->stem_;}

		void setConcave(cv::Mat concave){this->concave_ = concave;}
		void setCut(cv::Mat cut){this->cut_ = cut;}
		void setDepth(cv::Mat depth){this->depth_ = depth;}
		void setGray(cv::Mat gray){this->gray_ = gray;}
		void setK4(cv::Mat k4){this->k4_ = k4;}
		void setK6(cv::Mat k6){this->k6_ = k6;}
		void setStats(StatsRoi stats){this->stats_ = stats;}
		void setStem(cv::Mat stem){this->stem_ = stem;}

		friend ostream &operator << (ostream &cout, StatsResolver &r ){

			cout<< "area: " <<r.stats_.area << endl;
			cout<< "areaROI: " <<r.stats_.areaRoi << endl;
			cout<< "x: " <<r.stats_.x << endl;
			cout<< "y: " <<r.stats_.y << endl;
			cout<< "depthAVG: " <<r.stats_.depthAvg << endl;
			cout<< "depthDev: " <<r.stats_.depthDev << endl;
			cout<< "depthMin: " <<r.stats_.depthMin << endl;
			cout<< "depthMax: " <<r.stats_.depthMax << endl;

			cout<< "DistX: " <<r.stats_.distX<< endl;
			cout<< "DistY: " <<r.stats_.distY << endl;

			cout<< "k4AVG: " <<r.stats_.k4Avg << endl;
			cout<< "k4Dev: " <<r.stats_.k4Dev << endl;
			cout<< "k4Min: " <<r.stats_.k4Min << endl;
			cout<< "k4Max: " <<r.stats_.k4Max << endl;
			cout<< "k6AVG: " <<r.stats_.k6Avg << endl;
			cout<< "k6Dev: " <<r.stats_.k6Dev << endl;
			cout<< "k6Min: " <<r.stats_.k6Min << endl;
			cout<< "k6Max: " <<r.stats_.k6Max << endl;

			return cout;
		}
};


#endif

