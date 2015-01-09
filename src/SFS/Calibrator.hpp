#ifndef SETT
#define SETT
#define TIME_CHECK

//#include "Image.hpp"
//#include "SFS/Resolver.hpp"
//#include "ROD/Parameters.h"
#include "Resolver.hpp"
#include "../ROD/Parameters.h"

	class Calibrator{
		public:
			// Parametry analizy - w zasadzie typu SFSParameters
		Settings settings;

		// Dodatkowe pola: prze³¹czniki wizualizacji, monitorowania i utrwalania
		int visualSwitch; // 1: czy ma prezentowaæ okna on-line
		int monitorSwitch; // 1: czy ma siê zatrzymywaæ krok-po-kroku
		int logSwitch; // 1: cz
		
		/**
		*	@fn konstruktor z trybem klasyfikatora rozmytego
		*	@param light zrodlo swiatla 
		*	@param crop czy przycinac tlo i tworzyc maske tla 1=tak 0=nie
		*	@param gaussMat wymiar macierzy fitru Gaussa (zawsze NxN)
		*	@param gaussSig sigma filtru Gaussa
		*	@param iter ilosc iteracji algorytmu SFS
		*	@param thresholdArea okreslenie od jakiej wielkosci pola maja byc brane pod uwage ROI
		*	@param numClasses liczba klas obszarow ROI domyslnie 8
		*	@param fileMLP plik z siecia MLP
		*	@vis
		*	@monit debug 0=tryb zwykly 1=tryb debug
		*	@log
		*/
		Calibrator(SFSParameters spa, const char * fileMLP, int vis = 0, int debug = 0, int log = 0){
			try{
				this->checkParams(spa.light, spa.crop, spa.gaussMat, spa.gaussSig, 
											spa.iter, debug, spa.thresholdArea, spa.numClasses, fileMLP);
				this->initializeAnalysisAlgorithms(spa.light, spa.crop, spa.gaussMat, spa.gaussSig, 
											spa.iter, debug, spa.thresholdArea);
			//err
				this->initializeClassifierMLP(spa.numClasses, fileMLP);
			//~err
				this->settings.classifierMode = 1;  //0-fuzzy, 1-mlp
				this->settings.debugMode = debug;

				this->visualSwitch = vis;
				this->monitorSwitch = debug;
				this->logSwitch = log;

				this->resolver = new Resolver();
				
			}

			catch(cv::Exception &e){throw e;}
			catch(ATDAException &e){throw e;}
			catch(std::exception &e){throw e;}
		}

		/**
		*	@fn konstruktor z trybem klasyfikatora rozmytego
		*	@param light zrodlo swiatla 
		*	@param crop czy przycinac tlo i tworzyc maske tla 1=tak 0=nie
		*	@param gaussMat wymiar macierzy fitru Gaussa (zawsze NxN)
		*	@param gaussSig sigma filtru Gaussa
		*	@param iter ilosc iteracji algorytmu SFS
		*	@param thresholdArea okreslenie od jakiej wielkosci pola maja byc brane pod uwage ROI
		*	@param debug 0=tryb zwykly 1=tryb debug
		*	@param numClasses liczba klas obszarow ROI domyslnie 8
		*	@param fileSets plik ze zbiorami rozmytymi
		*	@param fileRules plik z plikami regul
		*/
		Calibrator(cv::Point3d light,int crop, int gaussMat, double gaussSig, 
						int iter, int thresholdArea, int debug, int numClasses, string fileRules, 
						string fileSets){
			this->initializeAnalysisAlgorithms(light, crop, gaussMat, gaussSig, 
										iter, debug, thresholdArea);
			this->settings.classifierMode = 0;  //0-fuzzy, 1-mlp
			this->initializeClassifierFuzzy(fileRules, fileSets, numClasses);
		}

		/**
		*	@fn konstruktor z trybem uczenia
		*	@param light zrodlo swiatla 
		*	@param crop czy przycinac tlo i tworzyc maske tla 1=tak 0=nie
		*	@param gaussMat wymiar macierzy fitru Gaussa (zawsze NxN)
		*	@param gaussSig sigma filtru Gaussa
		*	@param iter ilosc iteracji algorytmu SFS
		*	@param thresholdArea okreslenie od jakiej wielkosci pola maja byc brane pod uwage ROI
		*	@param debug 0=tryb zwykly 1=tryb debug
		*	@param numClasses liczba klas obszarow ROI domyslnie 8
		*	@param path sciezka bezwzgledna folderu rozwiazania
		*	@param fileRules plik z plikami regul
		*	@param fileParams plik z parametrami
		*	@param fileClasses plik z klasami
		*	@param fileSets plik ze zbiorami rozmytymi
		*	@param fileMLP plik z siecia MLP
		*
		*/
		Calibrator(cv::Point3d light,int crop, int gaussMat, double gaussSig, 
						int iter, int thresholdArea, int debug, int numClasses, 
						const wchar_t *path, string fileRules, string fileParams,
						string fileClasses, string fileSets, const char * fileMLP ){
			this->initializeAnalysisAlgorithms(light, crop, gaussMat, gaussSig, 
										iter, debug, thresholdArea);
			this->initializeTrainingMode(path, fileRules, fileClasses,
				fileParams, fileSets, numClasses, fileMLP);
			try{
			//	this->trainer = new Trainer(&this->settings);
			}
			catch( cv::Exception &e){throw e;}
			catch( ATDAException &e){throw e;}
			catch (std::exception &e){throw e;}
		}

		/*	
		*	@fn konstruktor domyslny z domyslnymi parametrami
		*/
		Calibrator(){
			this->settings.light.x = 0.001;
			this->settings.light.y = 0.001;
			this->settings.light.z = 1;
			this->settings.backgroundCrop = 1;
			this->settings.gaussMat = 11;
			this->settings.gaussSig = 11;
			this->settings.iter = 10;
			this->settings.storeMode = 0;
			this->settings.fileRules = "regulyTEST.csv";
			this->settings.fileClasses = "classesTEST.xml";
			this->settings.fileParams = "paramsTEST.xml";
			this->settings.fileClassesCSV = "klasyTEST.csv";
			this->settings.fileParamsCSV = "parametryTEST.csv";
			this->settings.fileSets = "zbioryxxxTEST.csv";
			this->settings.numClasses = 8;
			this->settings.fileMLP = "mlp.xml";
			this->settings.fileRules = "regulyTEST.csv";
			this->settings.fileSets = "zbioryxxxTEST.csv";
			this->settings.path = L"C:/SORTER/img/Database/01_Diffuse";
			this->loadMLP();
		}

		
		// Dwie wersje funkcji solution()
		// Wersja 1
		/**
		*	@fn funkcja uruchamia rozwiazanie dla podanej macierzy obrazu
		*	@param apple macierz obrazu
		*/
		vector<StatsRoi> solution(string aFile, cv::Mat& irimage, char outDir[], char outName[]){
			try{
				this->apple = irimage.clone();
				if(this->settings.debugMode==0){
					resolver->resolve(&(this->settings), apple);
				}
				else{
					resolver->debug(&(this->settings), this->visualSwitch, this->logSwitch, apple,
						aFile, outDir, outName);
				}
				vector<StatsRoi> rois = resolver->getRois();
				return rois;
			}
			catch(cv::Exception &e){throw e;}
			catch(ATDAException &e){throw e;}
			catch(std::exception &e){throw e;}
		}


		// Wersja 2
		/*
		*	@fn funkcja uruchamia rozwiazanie dla podanego pliku z obrazem
		*	@param appleFile: sciezka bezwgledna do pliku
		*/
		vector<StatsRoi> solution(string appleFile, char outDir[], char outName[]){
			try{
				this->apple = VisionProc::readImage(appleFile);

				if(this->settings.debugMode==0){
					resolver->resolve(&(this->settings), apple);
				}
				else{
					resolver->debug(&(this->settings), this->visualSwitch, this->logSwitch, apple, 
						appleFile, outDir, outName);
				//
				}
				// rozwi¹zanie
				vector<StatsRoi> rois = resolver->getRois();
				return rois; // wynik zwracany: lista opisów ROI
			}
			catch(cv::Exception &e){throw e;}
			catch(ATDAException &e){throw e;}
			catch(std::exception &e){throw e;}
		}

		//
		// Funkcje dostêpu getxxx()
		Settings getSettings(){return this->settings;}
		cv::Mat *getClasses(){return this->classes;}
		
		CvANN_MLP *getMlp(){return this->settings.mlp;}
		cv::Mat getRules(){return this->rules;}

		void setSettings(Settings settings){this->settings = settings;}
		void setMlp(CvANN_MLP *nn){this->settings.mlp = nn;}
		void setRules(cv::Mat rulesMat){this->rules = rulesMat;}

		//Calibrator::~Calibrator(){
		//	delete this->resolver;
		//}

private:
		cv::Mat *classes;
		cv::Mat rules;
		cv::Mat apple;
		Resolver *resolver;
		Image *img;
		//Trainer *trainer;

		/**
		*	@fn sprawdzenie poprawnosci wpisanych parametrow
		**/
		void checkParams(cv::Point3d light, int crop, int gaussMat, double gaussSig, 
						int iter, int thresholdArea, int debug, int numClasses, 
						const char * fileMLP){
			if(crop != 0 || crop != 1){

			}
			if(gaussMat % 2 == 0){
				throw  ATDAException(BAD_INITIALIZE_PARAM,
					"Calibrator::checkParams macierz filtru gaussa o wymiarze parzystym");
			}
			if(thresholdArea < 0 || thresholdArea > 1000){
				throw  ATDAException(BAD_INITIALIZE_PARAM,
					"Calibrator::checkParams progowanie o takim parametrze niedozwolone");
			}
			if(numClasses < 0 || numClasses > 9){
				throw ATDAException(BAD_INITIALIZE_PARAM,
					"Calibrator::checkParams liczba klas mniejsza od zera lub wieksza od 9");
			}
			
		}


		/**
		*	@fn inicjalizacja ustawien algorytmów analizy obrazow
		**/
		void initializeAnalysisAlgorithms(cv::Point3d light, int cropBkg, 
						int gaussMat, double gaussSig, int iter, int debug, 
						int thresholdArea ){
			this->settings.light.x = light.x;
			this->settings.light.y = light.y;
			this->settings.light.z = light.z;
			this->settings.backgroundCrop = cropBkg;
			this->settings.gaussMat = gaussMat;
			this->settings.gaussSig = gaussSig;
			this->settings.iter = iter;
			this->settings.storeMode = debug;
			this->settings.thresholdArea = thresholdArea;
			this->settings.thresholdConcave = 0.0009;
			this->settings.thresholdStem = -0.0033;
		}

		/**
		*	@fn inicjalizacja ustawien trybu uczenia
		*/
		void initializeTrainingMode(const wchar_t *path, string fileRules,
					string fileClasses, string fileParams, string fileSets,
					int numClasses, const char * fileMLP){
			this->settings.fileRules = fileRules;
			this->settings.fileClasses = fileClasses;
			this->settings.fileParams = fileParams;
			this->settings.fileClassesCSV = "klasyv1.csv";
			this->settings.fileParamsCSV = "parametryv1.csv";
			this->settings.fileSets = fileSets;
			this->settings.numClasses = numClasses;
			this->settings.fileMLP = fileMLP;
			this->settings.path = path;
		}

		/**
		*	@fn inicjalizacja ustawien dla mlp
		*
		*/
		void initializeClassifierMLP(int numClasses, const char * fileMLP){
			try{
				this->settings.numClasses = numClasses;
				this->settings.fileMLP = fileMLP;
				this->loadMLP();
			}
			catch(cv::Exception &e){throw e;}
			catch(ATDAException &e){throw e;}
			catch(std::exception &e){throw e;}
		}

		/**
		*	@fn inicjalizacja ustawien dla klasyfikatora rozmytego
		*
		*/
		void initializeClassifierFuzzy(string fileRules, string fileSets, int numClasses){
			this->settings.fileRules = "regulyTEST.csv";
			this->settings.fileSets = "zbioryxxxTEST.csv";
			this->settings.numClasses = 8;
		}
		

		/**
		*	@fn przygotowanie danych dla regul rozmytych
		*
		*/
		void loadClassifierFuzzyFiles(){
			try{
				this->loadRules();
				this->loadSets();
			}
			catch(cv::Exception &e){throw e;}
			catch(ATDAException &e){throw e;}
			catch(std::exception &e){throw e;}
		}
	

		/**
		*	@fn wczytanie nauczonej sieci
		*	
		*/
		void loadMLP(){
			this->settings.mlp = new CvANN_MLP;

			fstream file;
			file.open(this->settings.fileMLP, ios::in);
			if(!file.good()){
				std::cout<<"Error: load MLP\n";
				throw ATDAException(FILE_PROBLEM,
					"Calibrator::loadMLP nie mozna otworzyc pliku z mlp");
			}
			if(file.eof()){
				throw ATDAException(FILE_PROBLEM,
					"Calibrator::loadMLP plik z mlp pusty");
			}
			file.close();
			this->settings.mlp->load(this->settings.fileMLP);

		}


		/**
		*	@fn funkcja ³aduje regu³y z pliku
		*	
		*	 reprezentacja regu³:
		*				 ___param0___|___param1___|___param2___|__nastepnik__|
		*		regula0	|	  0	     |	   1      |	    1      |	  0      |
		*	    regula1	|		     |			  |		       |             |
		*						.........
		*		regula n|____________|____________|____________|_____________|
		**/
		void loadRules(){
			fstream file;
			string data;
			stringstream line;
			string tmp;
			cv::Mat row = cv::Mat::zeros(1, 19, CV_32FC1);
			int i;
			
			file.open(this->settings.fileRules.c_str(), ios::in);
			if(!file.good()){
				throw ATDAException(FILE_PROBLEM,
					"Calibrator::loadRules nie mozna otworzyc pliku z regulami");
			}
			if(file.eof()){
				throw ATDAException(FILE_PROBLEM,
					"Calibrator::loadRules plik z regulami pusty");
			}
			getline(file, data);			// wczytanie naglowka
			data.clear();

			while(!file.eof()){
				line.clear();
				getline(file, data);
				line << data;
				if(line.peek()==EOF){ 
					break;
				}
				else{
					for(i=0; i<20; i++){
						line >> tmp;
						if (tmp=="x" || atof(tmp.c_str()) < 0 ||
							atof(tmp.c_str())>this->settings.numClasses){
							row.at<float>(0,i) = 10.0;
						}
						else{
							row.at<float>(0,i) = (float)atof(tmp.c_str());
						}
					}
					this->rules.push_back(row);	
				}
			}
			file.close();
		}


		/** 
		*	@fn funkcja ³aduje opis zbiorów z pliku 
		*	
		*	 
		*					
		*	struktura jednej klasy:
		*				 ___a___|___b___|___c___|
		*		param0	|		|		|		|
		*	    param1	|		|		|		|
		*						.........
		*		param17	|_______|_______|_______|
		*	
		*	
		**/
		void loadSets(){
			fstream file;
			string data;
			float a, b, c;
			
			this->classes = new cv::Mat[this->settings.numClasses];
			int i,j;
			cv::Mat tmpRow(1, 3, CV_32FC1);

			file.open(this->settings.fileSets.c_str(), ios::in);
			
			if(!file.good()){
				throw ATDAException(FILE_PROBLEM,
					"Calibrator::loadSets nie mozna otworzyc pliku ze zbiorami");
			}
			if(file.eof()){
				throw ATDAException(FILE_PROBLEM,
					"Calibrator::loadSets plik ze zbiorami pusty");
			}

			getline(file, data);

			for(i= 0; i<this->settings.numClasses; i++){
				file >> data;
				for(j=0; j<18; j++){
					file >> a >> b >> c;
					tmpRow.at<float>(0,0) = a;
					tmpRow.at<float>(0,1) = b;
					tmpRow.at<float>(0,2) = c;
					classes[i].push_back(tmpRow);
				}

			}
			file.close();
		}

	};

#endif
