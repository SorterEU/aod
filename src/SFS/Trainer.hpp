#ifndef TRAINER
#define TRAINER

#include "Classifier.hpp"
//#include "Calibrator.hpp"
#include "Analizer.hpp"


class Trainer{

	public:
		/**
		*	@fn konstruktor klasy trenujacej siec neuronowa i zbierajacej dane
		*		
		**/
		Trainer(Settings *settings){
			try{
				this->prepareFolder(settings->path);
				/*
				this->prepareData(settings->iter, settings->filterWindow,
					settings->gaussMat, settings->gaussSig, settings->backgroundCrop,
					settings->light, settings->storeMode, settings->numClasses,
					settings->fileClasses, settings->fileParams, settings->fileSets, 
					settings->thresholdArea, settings->thresholdConcave, 
					settings->thresholdStem);
					*/
				this->mlpTrain(settings->fileMLP, settings->fileParams,
					settings->fileClasses);
				VisionProc::convertDataToCSV(settings->fileParams, 
					settings->fileClasses, settings->fileParamsCSV, 
					settings->fileClassesCSV);
			}
			catch( cv::Exception &e){throw e;}
			catch( ATDAException &e){throw e;}
			catch (std::exception &e){throw e;}
		}

		vector<string> getFileList(){return this->fileList;}
		Image *getApple(){return this->apple;}
		Analizer *getAnalizer(){return this->analizer;}

		void setFileList(vector<string> list){this->fileList = list;}
		void setApple(Image *app){this->apple = app;}
		void setAnalizer(Analizer *a){this->analizer = a;}

	private:
		vector<string> fileList;
		Image *apple;
		Analizer *analizer;
		
		
		/**
		*		@fn metoda inicjalizujaca klasy rozwi¹zania dla jednego pliku
		*		@param filename			podana sciezka bezwgledna pliku
		*		@param iter				liczba iteracji dla algorytmu SFS
		*		@param e				---DEPRECATED - zmiana na e=2
		*		@param gaussMat			rozmiar macierzy filtru Gaussa
		*		@param gaussSig			sigma filtru Gaussa
		*		@param cropBackground	---deprecated --  zmienna okreslaj¹ca czy wyci¹æ t³o (przeniesc do calibratora)
		*		@param light			wektor padania œwiat³a --->  mozliwe zamiana na obliczanie tilt/slant w calibratorze
		*		@param storeMode		-- ZMIENNA TESTOWA -- 
		*		@return					obszary wklesle z ich parametrami
		*/
		vector<StatsRoi> resolveOne(std::string filename, int iter, int e, 
			int gaussMat, double gaussSig, int cropBackground,
			cv::Point3d light, int storeMode, int areaThreshold, 
			double thresholdConcave, double thresholdStem){
			
			try{
				cv::Mat appleImg = VisionProc::readImage(filename);				//wczytanie obrazu za pomoca funkcji narzedziowych
				this->apple = new Image(appleImg, cropBackground);				//inicjalizacja obiektu klasy przechowujacej obraz
				this->analizer = new Analizer(this->apple, gaussMat,			//inicjalizacja obiektu klasy analizujacej obraz
											gaussSig, light, iter, 
											areaThreshold, thresholdConcave, 
											thresholdStem);				
				vector<StatsRoi> stats = this->analizer->getStats();			//wczytanie wynikow analizy

				if (storeMode==1){												//opcjonalny zapis wynikow do pliku -> analiza
					string folderPath = VisionProc::getPath(filename);			//wczytanie sciezki folderu z zapisem probek
					string filename = VisionProc::fileName(filename);			//wczytanie nazwy pliku dla ktorego maja byc probki
					this->storeResults(folderPath, filename);					//zapis w formie tekstowej
					this->saveForLearn(folderPath, filename);					//zapis w formie bmp
				}
				return stats;
			}
			catch( cv::Exception &e){throw e;}
			catch( ATDAException &e){throw e;}
			catch (std::exception &e){throw e;}
		}

		/*
		*	@fn zbieranie danych z podanego folderu do nauki klasyfikatorow
		*	@see resolveOne(string,  int, int, int, double, int, cv::Point3d, int)
		*	@param numClass liczba zdefiniowanych klas w klasyfikatorze
		*	@param fileClasses nazwa pliku z klasami
		*	@param fileParams nazwa pliku z parametrami probek
		*	@param fileSets nazwa pliku ze zbiorami rozmytymi
		*/
		void prepareData(int iter, double e, int gaussMat, 
						double gaussSig, int calibCrop, cv::Point3d light, 
						int storeMode, int numClass, string fileClasses,
						string fileParams, string fileSets, int areaThreshold, 
						double thresholdConcave, double thresholdStem){
			try{
				vector<string>::iterator it;										//iterator dla nazw plikow
				vector<StatsRoi> stats;												//parametry ROI
				cv::Mat training;													//macierz danych uczacych
				cv::Mat classes;													//macierz etykiet
				cv::Mat rowTraining =cv::Mat::zeros(1, 18, CV_32FC1);				//jedna probka danych uczacych
				cv::Mat rowLabel = cv::Mat::zeros(1, numClass, CV_32FC1);			//jedna probka etykiet
				int k;																//zmienna cin
				cv::Mat agg[8];														//tablica klas klasyfikatora rozmytego
				unsigned t0, elapsed;
				time_t start, end;
				for (it = this->fileList.begin(); it!=this->fileList.end(); it++){	//iteracja po kazdym pliku w podanym folderze
#ifdef DEBUG
					cout<< "Resolving: " << (*it) << "---------  " << endl;			
#endif
					t0=clock();														//zliczanie czasu
					start=time(0);
					stats = this->resolveOne((*it), iter, e, gaussMat, gaussSig,	//rozwiazanie dla jednego obrazu
											calibCrop, light, storeMode,
											areaThreshold, thresholdConcave, 
											thresholdStem);
					end=time(0);
					elapsed=clock()-t0;
					for(unsigned int i=0; i<stats.size(); i++){								//iteracja po kazdym ROI
						this->drawRoi(stats[i].rect);								//wyswietlenie ROI
						cout << "Typ: " << stats[i].type << "      ####" << endl;	//pomoc dla uczacego -> czy ROI jest brzegowe (typ1)
						rowTraining = assignParams(stats[i]);						//przypisanie parametrow do wektora
						//cout << rowTraining << endl;
						training.push_back(rowTraining);							//dodanie do macierzy danych uczacych
						k = -1;														//wyczyszczenie zmiennej k
					
						while( k-48<0 || k-48>numClass){							//jesli podana zla klasa 
							this->drawRoi(stats[i].rect);
							k = cv::waitKey(0);										//pobranie k od uzytkownika
							cout << k-48 << endl;
						}
						rowLabel = cv::Mat::zeros(1, numClass, CV_32FC1);			//todo ->zerowanie przez scalar
						rowLabel.at<float>(0, k-48) = 1.0;							//etykieta jako 1 w odpowiedniej kolumnie
						agg[k-48].push_back(rowTraining);							//dodanie wiersza danych uczacych do odpowiedniej klasy						
					
						classes.push_back(rowLabel);								//dodanie wiersza etykiet do macierzy etykiet
					}

					delete this->analizer;
					delete this->apple;
				
				}
		
				VisionProc::fileStoreMat(training, fileParams, "train");			//zapis danych uczacych
				VisionProc::fileStoreMat(classes, fileClasses, "classes");
				this->storeFuzzySets(agg, fileSets, numClass);
#ifdef TIME_CHECK
				cout << elapsed<< " ms   " ; 
				cout << end-start << " s" <<  endl;
#endif
			}
			catch( cv::Exception &e){throw e;}
			catch( ATDAException &e){throw e;}
			catch (std::exception &e){throw e;}
		}

		/**
		*	@fn metoda zapisujaca zbiory rozmyte do pliku
		*	@param agg tablica macierzy poszczegolnych klas 
		*	@param fileSets nazwa pliku zapisu zbiorow rozmytych
		*	@param numClass liczba klas
		*
		**/
		void storeFuzzySets(cv::Mat agg[8], string fileSets, int numClass){		//todo agg tworzone dynamicznie
			float a,b,c;
			int i,j;
			fstream file;
			double minVal, maxVal;							
			cv::Point minLoc, maxLoc;
			file.open(fileSets.c_str(), ios::out);										
		
			for(j=0; j<18; j++){
				file << "\t a \t b \t c \t" ;									// naglowki kolum pliku csv
			}
			file << endl;
			for (i=0; i<numClass; i++){
				if(agg[i].cols>0){
					file << "klasa" << i << "\t";								// naglowki wierszy pliku csv
					for(j=0; j<18; j++){
						cv::minMaxLoc(agg[i].col(j), &minVal, &maxVal,			// wyszukanie wartosci minimalnych i maksymalnych zbioru
									&minLoc, &maxLoc);
						a = (float)minVal;
						c = (float)maxVal;
						b = a + ((c-a)/2);
						file << (float)a << "\t" << (float)b << "\t" << (float)c << "\t";
						}
				file << endl;
				}
			}
			file.close();
		}


		/**
		*	@fn przypisanie wartosci statystyk ROI do macierzy(wektora)
		*	@param stats statystyki jednego ROI
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
		*	@fn trenowanie sieci neuronowej
		*	@param fileMLP nazwa pliku sieci
		*	@param fileParams nazwa pliku danych uczacych
		*	@param fileClasses nazwa pliku etykiet
		*
		**/
		void mlpTrain(const char *fileMLP, string fileParams, string fileClasses){

			CvTermCriteria criteria; //TODO: sterowanie przez settings
			criteria.max_iter = 100;											//ustawienia parametrow sieci
			criteria.epsilon = 0.00001f;
			criteria.type = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;	
			CvANN_MLP_TrainParams params;
			params.train_method = CvANN_MLP_TrainParams::BACKPROP;
			params.bp_dw_scale = 0.05f;
			params.bp_moment_scale = 0.05f;
			params.term_crit = criteria;

			CvANN_MLP mlp;

			cv::Mat layers = cv::Mat(3, 1, CV_32SC1);							//tworzenie warstw sieci				
			layers.row(0) = cv::Scalar(18);
			layers.row(1) = cv::Scalar(30);
		    //layers.row(2) = cv::Scalar(14);
			layers.row(2) = cv::Scalar(8);

			mlp.create(layers);

			cv::Mat training, classes;
			training.convertTo(training, CV_32FC1);
			classes.convertTo(classes, CV_32FC1);
			cv::FileStorage tr(fileParams, cv::FileStorage::READ);				//odczyt danych uczacych z pliku
			cv::FileStorage cl(fileClasses, cv::FileStorage::READ);
			tr["train"] >> training;
			cl["classes"] >> classes;
			tr.release();
			cl.release();
	
		    cv::Mat tmp;
			training(cv::Rect(0, 0, 18, training.rows)).copyTo(tmp);			//odciecie parametru typu

			mlp.train(tmp, classes, cv::Mat(), cv::Mat(), params);				//trening
			mlp.save(fileMLP);													//zapis sieci do pliku

		}


		/**
		*	@fn wyswietlanie aktualnego ROI
		*	@param roi ROI
		*
		**/
		void drawRoi(cv::Rect roi){
			cv::Mat rois = this->apple->getAppleImg().clone();						
			cv:: Rect bounding_rect;
			cv::RNG rng(12345);
			bounding_rect=roi;
			cv::Scalar color = cv::Scalar( rng.uniform(0, 255),		
									rng.uniform(0,255), rng.uniform(0,255) );
			rectangle(rois, bounding_rect,  color ,1, 8,0);
			imshow("rois", rois);
		}


		/**
		*	@fn pomocnicze przygotowanie folderu o podanej sciezce
		*	@param sDir sciezka folderu obrazow uczacych
		**/
		void prepareFolder(const wchar_t *sDir){
			this->ListDirectoryContents(sDir);
			cout << this->fileList[0] << endl;
			this->prepareFiles();
		}


		/**
		*	@fn przeglad katalogow w katalogu o podanej sciezce (WINDOWS!)
		*	@param sDir sciezka katalogu nadrzednego z obrazami uczacymi
		*
		**/
		bool ListDirectoryContents(const wchar_t *sDir){ 
			WIN32_FIND_DATA fdFile; 
			HANDLE hFind = NULL; 

			wchar_t sPath[2048]; 
			//Specify a file mask. *.* = We want everything! 
			wsprintf(sPath, L"%s\\*.*", sDir); 

			if((hFind = FindFirstFile(sPath, &fdFile)) == INVALID_HANDLE_VALUE){ 
				wprintf(L"Path not found: [%s]\n", sDir); 
				return false; 
			} 
			do{ 
				//Find first file will always return "."
				//    and ".." as the first two directories. 
				if(wcscmp(fdFile.cFileName, L".") != 0
						&& wcscmp(fdFile.cFileName, L"..") != 0) { 
					//Build up our file path using the passed in 
					//  [sDir] and the file/foldername we just found: 
					wsprintf(sPath, L"%s\\%s", sDir, fdFile.cFileName); 
					//Is the entity a File or Folder? 
					if(fdFile.dwFileAttributes &FILE_ATTRIBUTE_DIRECTORY){ 
						//wprintf(L"Directory: %s\n", sPath); 
						ListDirectoryContents(sPath); //Recursion, I love it! 
					} 
					else{ 
						//wprintf(L"File: %s\n", sPath); 
						wstring basicstring(sPath);
						//basicstring += L" (basic_string)";
						string str( basicstring.begin(), basicstring.end() );
						this->fileList.push_back(str);
					} 
				}
			} 
			while(FindNextFile(hFind, &fdFile)); //Find the next file. 

			FindClose(hFind); //Always, Always, clean things up! 

			return true; 
		} 

		/**
		*	@fn pomocnicze przygotowanie katalogow z obrazami uczacymi
		*
		**/
		void prepareFiles(){
			vector<string>::iterator it;

			for (it = this->fileList.begin(); it!=this->fileList.end(); it++){
				if( (*it)[(*it).rfind('.')-1] != '0'){
					it = this->fileList.erase(it);
					it--;
				}
				else if((*it).rfind("DepthMaps")!=std::string::npos ||
						(*it).rfind("FactorsK")!=std::string::npos ||
						(*it).rfind("SurfMap")!=std::string::npos){
					it = this->fileList.erase(it);
					it--;
				}
				else{
					VisionProc::ReplaceStringInPlace((*it), "\\", "/");
					VisionProc::ReplaceStringInPlace((*it), "\\\\", "/");
				}
			}

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
			cv::imwrite(name, this->analizer->getColorMap());
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
			cv::imwrite(name, this->analizer->getColorMap());
		}

		/**
		*	@fn pomocniczy zapis macierzy w formie tekstowej
		*	@param results	macierz do zapisu
		*	@param fullpath	sciezka zapisu
		**/
		void storeMat(cv::Mat results, string fullpath){
			fstream file;
			file.open(fullpath, ios::out);
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
