#ifndef RESOLVER
#define RESOLVER

#include "Classifier.hpp"
#include "Analizer.hpp"
//#include "Trainer.hpp"

class Resolver{
	public:
		
	
		/**
		*	@fn konstruktor rozwiazania dla podanej macierzy obrazu
		*	@param settings ustawienia
		*	@param image macierz obrazu
		*
		**/
		Resolver(){
			
		}

		/**
		*	@fn rozwiazanie klasyfikacji
		*	@param settings ustawienia
		*	@param image podany obraz
		*
		*/
		void resolve(Settings *settings, cv::Mat image){
			// Pomiar czasu
			unsigned t0, elapsed;
			time_t start, end;
			try{
				t0=clock();		// zliczanie czasu
			    start=time(0);
				
				this->apple = new Image(image, settings->backgroundCrop);		//nowy obraz
				this->analizer = new Analizer(this->apple, settings->gaussMat,	//nowy analizer - wykonuje ananlizê 3D
					settings->gaussSig,	settings->light, settings->iter,	
					settings->thresholdArea, settings->thresholdConcave,
					settings->thresholdStem );
				// koniec rekonstrukcji SFS

				end=time(0);
				elapsed=clock()-t0;
				cout << "Czas wykonania analizy 3D (10-12a):" << endl;
				cout << elapsed << " ms" <<  endl;
				cout << end-start << " s" <<  endl;
	            
				// pocz¹tek zliczania czasu
				t0=clock();	
			    start=time(0);
				vector<StatsRoi> stats = this->analizer->getStats();		//gotowe roi
				//TEST
				//cout<< "Po getStats" << endl;
				//
				if (settings->classifierMode == 0){
					this->fuzzyPrepare(settings, stats);			//klasyfikator fuzzy
				}
				else{
					this->mlpPrepare(settings, stats);			//klasyfikatorMLP
				}
				//TEST
				//cout<< "Po mlpPrepare" << endl;
				//

				this->selectRois();
				//TEST
				//cout<< "Po selectRois" << endl;
				//
				// koniec pomiaru
				end=time(0);
				elapsed=clock()-t0;
				cout << "Czas klasyfikacji 3D (12 b):" << endl;
				cout << elapsed << " ms" <<  endl;
				cout << end-start << " s" <<  endl;
			}
			catch(cv::Exception &e){throw e;}
			catch(ATDAException &e){throw e;}
			catch(std::exception &e){throw e;}
		}


		/**
		*	@fn funkcja wspomagajaca debugowanie algorytmów SFS
		*	@param settings: ustawienia
		*	@param image: obraz do analizy
		*	@path sciezka: zapisu plikow wyjscia
		*
		*/
		void debug(Settings *settings, int visual, int logs, cv::Mat image, 
			string path, char ouDir[], char ouName[]){
			
			try{
				this->apple = new Image(image, settings->backgroundCrop);
				cout << "SFS Etap 9: Generowanie obrazow i przekszta³ceñ" << endl;
	            //cout << "(nacisnij przycisk, aby przejsc dalej lub" << endl;
				//cout <<	"- spacja, aby wyczyscic wyniki poprzedniego etapu )" << endl;
				char outName[120];
				cv::Mat tmp = this->apple->getAppleImg().clone();
				if (visual == 1) {
					cv::circle(tmp, this->apple->getMassCenter(), 3,
							cv::Scalar(255, 0, 0), 2);
					imshow("S9.1 Obraz oryginalny ze srodkiem masy",tmp);
					if (logs == 1) { 
						sprintf(outName, "%s/3D_1_center%s.png", ouDir, ouName);
		        		cv::imwrite(outName, tmp);
					}
				}
				
				tmp = this->apple->getAppleGray().clone();
				cv::GaussianBlur(tmp, tmp, cv::Size(settings->gaussMat,
								settings->gaussMat), settings->gaussSig,
								cv::BORDER_DEFAULT);
				
				if (visual == 1) {
					imshow("S9.2 Obraz po filtracji Gaussa", tmp);
					imshow("S9.3 Maska obiektu", this->apple->getAppleMask());
					if (logs == 1) { 
						sprintf(outName, "%s/3D_2_gauss%s.png", ouDir, ouName);
		        		cv::imwrite(outName, tmp);
						sprintf(outName, "%s/3D_3_mask%s.png", ouDir, ouName);
		        		cv::imwrite(outName, this->apple->getAppleMask());
					}
				}
				//int k = cv::waitKey();
				//if(k == 32){cv::destroyAllWindows();}

				cout << "SFS Etapy 10-12: Analiza 3d" << endl;
				this->analizer = new Analizer(this->apple, settings->gaussMat,
					settings->gaussSig,	settings->light, settings->iter, 
					settings->thresholdArea, settings->thresholdConcave,
					settings->thresholdStem );
				
				if (visual == 1) {
					imshow("S10 Mapa znalezionych wg³êbieñ", this->analizer->getColorMap());
					if (logs == 1) { 
						sprintf(outName, "%s/3D_4_holes%s.png", ouDir, ouName);
		        		cv::imwrite(outName, this->analizer->getColorMap() );
					}
				}
				
				//
				vector<StatsRoi> stats = this->analizer->getStats();
				
				string folderPath = VisionProc::getPath(path);
				string filename = VisionProc::fileName(path);
				
				//this->storeResults(folderPath, filename);
				//this->saveForLearn(folderPath, filename);

				if (settings->classifierMode == 0){				// klasyfikacja
					this->fuzzyPrepare(settings, stats);
				}
				else{
					this->mlpPrepare(settings, stats, logs, ouDir, ouName );
				}

				//TEST
				//cout<<"debug Po mlpPrepare"<< endl;
				//
				this->selectRois();
				//TEST
				//cout<<"debug Po selectRois"<< endl;
				//cv::destroyAllWindows();
			}
			catch(cv::Exception &e){throw e;}
			catch(ATDAException &e){throw e;}
			catch(std::exception &e){throw e;}
		}

		Image *getApple(){return this->apple;}
		Analizer *getAnalizer(){return this->analizer;}
		vector <StatsRoi> getRois(){return this->rois;}

		void setApple(Image *app){this->apple = app;}
		void setAnalizer(Analizer *a){this->analizer = a;}
		void setRois(vector<StatsRoi> rois){this->rois = rois;}
	
	private:
		Image *apple;
		Analizer *analizer;
		vector <StatsRoi> rois;
		vector <StatsRoi> allStats;

		/**
		*	@fn wybranie roi po klasach wyjsciowych (usuniecie klas wewnetrznych biblioteki)
		*	
		*/
		void selectRois(){
			try{
				cv::Mat selectedClasses = cv::Mat::zeros(1, 6, CV_32FC1);
				cv::Point minLoc, maxLoc;
				vector<StatsRoi> selectedRois;
				unsigned int i;
				double minVal, maxVal;
				
				//TEST
				cout<< allStats.size() << endl;
				//

				for(i=0; i<this->allStats.size(); i++){

					//ERROR: tu wystêpuje za drugim razem b³¹d braku obiektu Mat!
					if ( (this->allStats[i].response).dims < 2)
						break;
					//

					cv::minMaxLoc(this->allStats[i].response, &minVal, &maxVal, 
						&minLoc, &maxLoc);		//wybranie po najwiekszej wartosci

					this->allStats[i].selected = cv::Mat::zeros(1, 6, CV_32FC1);
					
					this->allStats[i].selected = this->allStats[i].response(
						cv::Range(0,1), cv::Range(1,7) ).clone();
					//TEST
					//cout<< i <<":w pêtli allStats" << endl;
					//
					if(maxLoc.x != 0 && maxLoc.x != 7){
						selectedRois.push_back(this->allStats[i]);
					}
				}
				//TEST
				cout<< "po pêtli allStats" << endl;
				//
				this->rois = selectedRois;
			}
			catch(cv::Exception &e){throw e;}
			catch(ATDAException &e){throw e;}
			catch(std::exception &e){throw e;}
		}


		/**
		*	@fn wyswietlenie wszystkich ROI na obrazie
		*	
		*
		**/
		void drawAllRois(int logs, char ouDir[], char ouName[]){

			char outName[120];
			cv::Mat rois = this->apple->getAppleImg().clone();
			vector<StatsRoi> bounding_rect;
			cv::RNG rng(12345);

			cv::Scalar color = cv::Scalar( rng.uniform(0, 255), 
									rng.uniform(0,255), rng.uniform(0,255) );
			for(int i=0; i<this->allStats.size(); i++){
				rectangle(rois, this->allStats[i].rect,  color, 1, 8,0);
			}

			imshow("S11. Obszary rois", rois);
			if (logs == 1) { 
				sprintf(outName, "%s/3D_6_rois%s.png", ouDir, ouName);
				cv::imwrite(outName, rois );
			}
		}


		/**
		*	@fn wyswietlenie ROI na obrazie
		*	@param roi ROI do wyswietlenia
		*
		**/
		void drawRoi(cv::Rect roi){

			cv::Mat rois = this->apple->getAppleImg().clone();
			cv::Rect bounding_rect;
			cv::RNG rng(12345);

			bounding_rect=roi;
			cv::Scalar color = cv::Scalar( rng.uniform(0, 255), 
									rng.uniform(0,255), rng.uniform(0,255) );
			rectangle(rois, bounding_rect,  color, 1, 8,0);
			imshow("S11. Obszar roi", rois);
		}


		/**
		*	@fn przygotowanie danych do klasyfikatora rozmytego
		*	@param settings ustawienia
		*	@param stats wektor ROI z ich parametrami
		**/
		void fuzzyPrepare(Settings *settings, vector<StatsRoi> stats){
			try{
				Classifier *fuzzy; 
				cv::Mat params;
				fuzzy = new Classifier(settings);
				cv::Mat response = cv::Mat::zeros(1, settings->numClasses,			//miejsce na odpowiedz klasyfikatora
									CV_32FC1);
					for(unsigned int i=0; i<stats.size(); i++){								//iteracja po kazdym roi
						fuzzy->fuzzyPredict(settings, stats[i]);
						response = fuzzy->getResponse();							//odpowiedz
						this->allStats.push_back(stats[i]);
						allStats[i].response = response.clone();
						if(settings->debugMode==1){
							params = this->assignParams(stats[i]);
							//cout << "Parametry ROI: " << endl;
							//cout << params << endl;
							this->drawRoi(stats[i].rect);
							//cout << "Klasyfikacja: "<< endl;
							//cout << response << endl;
						//cv::waitKey(0);
					}
					
					}
					delete fuzzy;
					delete this->analizer;
					delete this->apple;
			}
			catch( cv::Exception &e){throw e;}
			catch(std::exception &e){throw e;}
		}

		/**
		*	@fn przygotowanie danych do sieci neuronowej
		*
		**/
		void mlpPrepare(Settings *settings, vector<StatsRoi> stats, int logs=0, char ouDir[]="\0", char ouName[]="\0"){
			try{
				Classifier *mlp; 
				mlp = new Classifier(settings->mlp);
				cv::Mat sample(1, 18, CV_32FC1);								//przygotowanie probki
				cv::Mat response = cv::Mat::zeros(1, settings->numClasses,		//alokacja odpowiedzi
									CV_32FC1);
				cv::Mat params; 
				for(unsigned int i=0; i<stats.size(); i++){						//iteracja po kazdym roi
					sample = assignParams(stats[i]);							//przypisanie parametrow do probki
					mlp->mlpPredict(sample);					
					response = mlp->getResponse();								//odpowiedz sieci
					this->allStats.push_back(stats[i]);
					this->allStats[i].response = response.clone();		
					// Wynik
					if(settings->debugMode==1){
						params = this->assignParams(stats[i]);
						//cout << "Parametry ROI: " << endl;
						//cout << params << endl;

						//this->drawAllRois();
						//cout << "Klasyfikacja: "<< endl;
						//cout << response << endl;
						//cv::waitKey(0);
					}
				
				}

				if(settings->debugMode==1){
					this->drawAllRois(logs, ouDir, ouName);
				}

				delete mlp;
				delete this->analizer;
				delete this->apple;
			}
			catch( cv::Exception &e){throw e;}
			catch(std::exception &e){throw e;}
		}


		/**
		*	@fn przypisanie parametrow probki do macierzy (wektora)
		*	@param stats jedna probka
		*
		**/
		cv::Mat assignParams(StatsRoi stats){
			cv::Mat params = cv::Mat::zeros(1, 18, CV_32FC1);					// 18 parametrow
			params.at<float>(0, 0) = (float)stats.area;							//przypisanie parametrow do wiersza probki
			params.at<float>(0, 1) = (float)stats.areaRoi;
			params.at<float>(0, 2) = stats.depthAvg;
			params.at<float>(0, 3) = stats.depthDev;
			params.at<float>(0, 4) = stats.depthMax;
			params.at<float>(0, 5) = stats.depthMin;
			params.at<float>(0, 6) = (float)stats.distX;
			params.at<float>(0, 7) = (float)stats.distY;
			params.at<float>(0, 8) = stats.k4Avg;
			params.at<float>(0, 9) = stats.k4Dev;
			params.at<float>(0, 10) = stats.k4Max;
			params.at<float>(0, 11) = stats.k4Min;
			params.at<float>(0, 12) = stats.k6Avg;
			params.at<float>(0, 13) = stats.k6Dev;
			params.at<float>(0, 14) = stats.k6Max;
			params.at<float>(0, 15) = stats.k6Min;
			params.at<float>(0, 16) = (float)stats.x;
			params.at<float>(0, 17) = (float)stats.y;
			return params;
		}



		/**
		*	@fn zapis danych wyjsciowych algorytmow w celu debugowania w formacie tesktowym
		*	@param path sciezka zapisu
		*	@param filename nazwa pliku danych wyjsciowych
		*
		**/
		void storeResults( string path, string filename){
			this->storeMat(this->analizer->getDepthMap(), path +"DepthMaps/"+filename+".mat");
			this->storeMat(this->analizer->getApproxMap(), path +"DepthMaps/"+filename+"approx.mat");
			this->storeMat(this->analizer->getK()[0], path +"FactorsK/"+filename+"_k1.mat");
			this->storeMat(this->analizer->getK()[1], path +"FactorsK/"+filename+"_k2.mat");
			this->storeMat(this->analizer->getK()[2], path +"FactorsK/"+filename+"_k3.mat");
			this->storeMat(this->analizer->getK()[3], path +"FactorsK/"+filename+"_k4.mat");
			this->storeMat(this->analizer->getK()[4], path +"FactorsK/"+filename+"_k5.mat");
			this->storeMat(this->analizer->getK()[5], path +"FactorsK/"+filename+"_k6.mat");
			string name = path +"SurfMap/"+filename+".bmp";
			//cv::imwrite(name, this->analizer->getColorMap());
		}


		/**
		*	@fn zapis danych wysciowych algorytmow analizy w formacie bmp
		*	@see storeResults(string, string)
		**/
		void saveForLearn( string path, string filename){
			cv::imwrite(path +"DepthMaps/"+filename+".bmp", this->analizer->getDepthMap());
			cv::imwrite(path +"DepthMaps/"+filename+"approx.bmp", this->analizer->getApproxMap());
			cv::imwrite(path +"FactorsK/"+filename+"_k1.bmp", this->analizer->getK()[0]);
			cv::imwrite(path +"FactorsK/"+filename+"_k2.bmp", this->analizer->getK()[1]);
			cv::imwrite(path +"FactorsK/"+filename+"_k3.bmp", this->analizer->getK()[2]);
			cv::imwrite(path +"FactorsK/"+filename+"_k4.bmp", this->analizer->getK()[3]);
			cv::imwrite(path +"FactorsK/"+filename+"_k5.bmp", this->analizer->getK()[4]);
			cv::imwrite(path +"FactorsK/"+filename+"_k6.bmp", this->analizer->getK()[5]);
			string name = path +"SurfMap/"+filename+".bmp";
			//cv::imwrite(name, this->analizer->getColorMap());
		}

		/**
		*	@fn pomocniczy zapis macierzy w formie tekstowej
		*	@param results	macierz do zapisu
		*	@param fullpath	sciezka zapisu
		**/
		void storeMat(cv::Mat results, string fullpath){
			fstream file;
			file.open(fullpath.c_str(), ios::out);
			if( file.good() == true ){
				for(int i=0; i<results.rows; i++){
					for(int j=0; j<results.cols; j++){
						file << results.at<double>(i,j);
						if( j!=results.cols - 1){
							file << "\t" ;
						}
					}
					file << endl; 
				}
			}
			file.close();
		}


};



#endif
