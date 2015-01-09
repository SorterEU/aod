#ifndef CLASSIFIER
#define CLASSIFIER
#include "Analizer.hpp"



class Classifier{


	public:

		/**DEPRECATED
		*	@fn konstruktor klasy klasyfikujacej zaglebienia 
			dla klasyfikatora rozmytego
		*	@param settings ustawienia
		*	@param stats statystyki zaglebienia
		*/
		Classifier(Settings *settings ){
			this->loadSetsFromFile(settings->numClasses, settings->fileSets); 
	        this->loadRulesFromFile(settings->fileRules);
			
		}

		/**
		*	@fn konstruktor klasy klasyfikujacej zaglebienia 
			dla klasyfikatora rozmytego
		*	@param settings ustawienia
		*	@param classesMat tablica macierzy zbiorow rozmytych w klasach
		*	@param rulesMat macierz regul rozmytych
		*/
		Classifier(Settings *settings, cv::Mat *classesMat, cv::Mat rulesMat){
			this->classes = classesMat;
			this->rules = rulesMat;

		}

		/**
		*	@fn kontruktor klasy 
		*	
		*/
		Classifier(){
			
		}


		/**
		*	@fn kontruktor klasy dla klasyfikatora mlp
		*	
		*/
		Classifier(CvANN_MLP *nn){
			this->mlp = nn;
		}

	

		/**DEPRECATED
		*	@fn funkcja klasyfikujaca z uzyciem sieci mlp
		*	@param sample probka do klasyfikacji
		*	@param fileMLP nazwa pliku z wytrenowana siecia
		*
		*/
		void mlpPredict(cv::Mat sample, const char *fileMLP){
			CvANN_MLP mlp;
			mlp.load(fileMLP);
			mlp.predict(sample, this->response);
		}


		/**
		*	@fn funkcja klasyfikujaca z uzyciem sieci mlp
		*	@param sample probka do klasyfikacji
		*	
		*
		*/
		void mlpPredict(cv::Mat sample){
			try{
				this->mlp->predict(sample, this->response);
			}
			catch(cv::Exception &e){throw e;}
			catch(ATDAException &e){throw e;}
			catch(std::exception &e){throw e;}
		}


		void fuzzyPredict(Settings *settings, StatsRoi stats){
			try{
				this->aggregateSets(settings->numClasses, stats);
				this->applyRules(settings->numClasses);
			}
			catch(cv::Exception &e){throw e;}
			catch(ATDAException &e){throw e;}
			catch(std::exception &e){throw e;}
		}

		cv::Mat getFuzzyValues(){return this->fuzzyValues;}
		cv::Mat *getClasses(){return this->classes;}
		cv::Mat getRules(){return this->rules;}
		cv::Mat getResponse(){return this->response;}
		CvANN_MLP *getMlp(){return this->mlp;}

		void setGetFuzzyValues(cv::Mat fuzzyV){this->fuzzyValues = fuzzyV;}
		void setClasses(cv::Mat *cl){this->classes = cl;}
		void setRules(cv::Mat rulesMat){this->rules = rulesMat;}
		void setResponse(cv::Mat responseMat){this->response = responseMat;}
		void setMlp(CvANN_MLP *nn){this->mlp = nn;}

	private:
		cv::Mat fuzzyValues;
		cv::Mat *classes;							//w zale¿noœci od liczby klas
		cv::Mat rules;
		cv::Mat response;
		CvANN_MLP *mlp;



		/** 
		*	@fn	funkcja rozmywaj¹ca zmienn¹ wejœciow¹
		*	@param a pocz¹tek przedzia³u zbioru rozmytego,
		*	@param b œrodek zbioru,
		*	@param c koniec zbioru
		*	@param x zmienna do rozmycia
		**/
		float fuzzify(float a, float b, float c, float x){
			if( (b-a)==0 || (b-c) == 0){
				throw ATDAException(BAD_FUZZY_SET, 
					" srodek zbioru rowny koncowi/poczatkowi");
			}
			float y;
			a = a-(0.05f*abs(a));													//todo: tolerance 00.5 sterowane settingsami
			c = c+(0.05f*abs(c));													//rozszerzenie zbioru o zadany zakres
			if(x >= a && x <=b){												//rozmywanie
				y = (x-a)/(b-a);
			}
			else if(x >b && x<=c){
				y = (x-c)/(b-c);
			}
			else{
				y = 0;
			}
			return y;
		}


		/**
		*	@fn zastosowanie regul
		*	@param numClasses zadana liczba klas
		**/
		void applyRules(int numClasses){
			try{
				cv::Mat oneRule = cv::Mat::zeros(1, this->rules.cols-1, CV_32FC1);	//do zbierania wynikow pojedynczych regul
				cv::Mat outRules = cv::Mat::zeros(this->rules.rows, 1, CV_32FC1);	//wyjœcia dla poszczegolnych regul
				cv::Mat aggregatedClasses = cv::Mat::zeros(numClasses, 1, CV_32FC1);//wyjscie z maksymalizacji regul dla klas wyjsciowych

				float tmp;
				int i,j;
				float paramClass;
				float fuzzyVal;
				double minValue, maxValue;
				cv::Point minLoc, maxLoc;
				vector<vector<float> > maximize(numClasses);
			
				for(i=0; i<rules.rows; i++){										// cols-1 bo ostatni wiersz to nastêpniki regul
					for(j=0; j<rules.cols-1; j++){
						paramClass = this->rules.at<float>(i,j);
						if(paramClass >=0 || paramClass <numClasses){
							fuzzyVal = this->fuzzyValues.at<float>(j, (int)paramClass);		// tu moze byæ blad z plikiem
							oneRule.at<float>(0, j) = fuzzyVal;
						}
					
					}
					cv::minMaxLoc(oneRule, &minValue, &maxValue, &minLoc, &maxLoc);
					outRules.at<float>(i,0) = (float)minValue;						//zastosowanie wnioskowania Mamdani
				}
				for(i=0; i<rules.rows; i++){
					tmp = rules.at<float>(i, rules.cols-1);
					maximize[(int)tmp].push_back(outRules.at<float>(i,0));
				
				}
				for(i=0; i<numClasses; i++){
					aggregatedClasses.at<float>(i, 0) =
							*max_element(maximize[i].begin(), maximize[i].end());	//maksymalizacja regul dotyczacych tych samych klas wyjsciowych

				}
				this->response = aggregatedClasses;
			}
			catch(cv::Exception &e){throw e;}
			catch(ATDAException &e){throw e;}
			catch(std::exception &e){throw e;}
		}


		/**
		*	@fn funkcja agreguj¹ca wszystkie wartoœci rozmyte
		*	reprezentacja:
		*				 ___klasa0___|___klasa1___|___klasa2___| ....
		*		param0	|	0.5	     |		0.3   |		       |
		*       param1	|	     	 |		      |		       |
		*					           	.........
		*		param17	|____________|____________|____________|
		*	@param numClasses zadana liczba klas
		*	@param stats roi wraz z parametrami do zbadania
		*	
		**/
		void aggregateSets(int numClasses, StatsRoi stats){
			try{
				cv::Mat fuzzys(18, numClasses, CV_32FC1);
				int j,i;
				float a, b, c;
				float y;
			
				vector<float> statsVec = vectorizeStats(stats);

				cout << "stats *** " << endl;
				for(j=0; j<18; j++){
					for(i=0; i<numClasses; i++){
						a = this->classes[i].at<float>(j,0);
						b = this->classes[i].at<float>(j,1);
						c = this->classes[i].at<float>(j,2);

						//cout << "klasa " << i << " a: " << a << " b: " << b << " c: " << c << endl; 
						y = fuzzify(a, b, c, statsVec.at(j));
						//cout << y << endl;
						fuzzys.at<float>(j,i) = y;
					}
				}
				//cout << statsVec.at(6) <<  endl;
				this->fuzzyValues = fuzzys.clone();
				cout << endl << endl << "fuzzy val :-------" << endl;
				cout << fuzzyValues<< endl;

			}
			catch(cv::Exception &e){throw e;}
			catch(ATDAException &e){throw e;}
			catch(std::exception &e){throw e;}
		}


		/**
		*	@fn funkcja zapisuje statystyki w postaci wektora
		*	@param stats zadane statystyki
		*
		**/
		vector<float> vectorizeStats(StatsRoi stats){
			vector<float> statsVec;
			statsVec.push_back((float)stats.area);			//0
			statsVec.push_back((float)stats.areaRoi);		//1
			statsVec.push_back(stats.depthAvg);				//2
			statsVec.push_back(stats.depthDev);				//3
			statsVec.push_back(stats.depthMax);				//4
			statsVec.push_back(stats.depthMin);				//5
			statsVec.push_back((float)stats.distX);			//6
			statsVec.push_back((float)stats.distY);			//7
			statsVec.push_back(stats.k4Avg);				//8
			statsVec.push_back(stats.k4Dev);				//9
			statsVec.push_back(stats.k4Max);				//10
			statsVec.push_back(stats.k4Min);				//11
			statsVec.push_back(stats.k6Avg);				//12
			statsVec.push_back(stats.k6Dev);				//13
			statsVec.push_back(stats.k6Max);				//14
			statsVec.push_back(stats.k6Min);				//15
			statsVec.push_back((float)stats.x);				//16
			statsVec.push_back((float)stats.y);				//17

			return statsVec;

		}

		/** DEPRECATED
		*	@fn funkcja ³aduje opis zbiorów z pliku 
		*	@param fileSets plik podany w configu
		*	 
		*					
		*	struktura jednej klasy:
		*				 ___a___|___b___|___c___|
		*		param0	|		|		|		|
		*	    param1	|		|		|		|
		*						.........
		*		param17	|_______|_______|_______|
		*	@param numClasses zadana liczba klas
		*	
		**/
		void loadSetsFromFile(int numClasses, string fileSets){
		
			fstream file;
			string data;
			float a, b, c;
			this->classes = new cv::Mat[numClasses];
			int i,j;
			cv::Mat tmpRow(1, 3, CV_32FC1);

			file.open(fileSets.c_str(), ios::in);
			getline(file, data);

			for(i= 0; i<numClasses; i++){
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


		/** DEPRECATED
		*	@fn funkcja ³aduje regu³y z pliku
		*	@param fileRules plik z regulami
		*	 reprezentacja regu³:
		*				 ___param0___|___param1___|___param2___|__nastepnik__|
		*		regula0	|	  0	     |	   1      |	    1      |	  0      |
		*	    regula1	|		     |			  |		       |             |
		*						.........
		*		regula n|____________|____________|____________|_____________|
		**/
		void loadRulesFromFile(string fileRules){

			fstream file;
			string data;
			stringstream line;
			string tmp;
			
			cv::Mat row = cv::Mat::zeros(1, 19, CV_32FC1);
			int i;
			 
			file.open(fileRules.c_str(), ios::in);
			getline(file, data);												// wczytanie naglowka
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
							atof(tmp.c_str())>9){
							row.at<float>(0,i) = 10.0;
						}
						else{
							row.at<float>(0,i) = (float)atof(tmp.c_str());
						}
					
					}
					this->rules.push_back(row);	
				}
				
			}
			//cout << rules << endl;
			//cout << rules.size() << endl;
			file.close();
		}

		
};

#endif
