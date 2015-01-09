#ifndef ANALYSIS
#define ANALYSIS

//#include "ROD/Parameters.h"
//#include "ROD/Properties.h"
//#include "ROD/UserParameters.h"
//#include "ROD/ResultOD.h"
//#include "ROD/RodFunctions.hpp"

#include "Parameters.h"
#include "Properties.h"
#include "UserParameters.h"
#include "ResultOD.h"

#include "RodFunctions.hpp"

#include "../STEM/stem_descriptors.h"

#include "../SFS/Calibrator.hpp"

#include <opencv/cv.h>
//#include <cv.h>
#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>  

using namespace std;

//extern namespace STEM;

//using namespace STEM;

extern FeatureDescriptor SzypulkaWObrysie(char* ouDir, char* ouName, int visual, int logs, cv::Mat imageRGB, 
							float relDetThresh = 0.11f, int backThresh= 20, float selStemPR= 0.8f);

extern FeatureDescriptor OdstajacaSzypulka(char* fileName, int visual, int logs, cv::Mat imageRGB);


namespace ROD{

	// "Rozci¹ganie histogramu"
	void HistoStretching(int a0, int A, int B, int a1, ::cv::Mat &MImg, ::cv::Mat &NormMImg)
	{
		int my=MImg.rows;
		int mx=MImg.cols;
		int val, nval;

		NormMImg = MImg.clone();
		double wsp =  ((double)(a1 - a0)) / ((double)(B - A));


		for (int j=0; j<mx; j++) { 
			for (int i=0; i<my; i++) {
				val = (int) MImg.at<uchar>(j, i);
				if (val <= A) 
					NormMImg.at<uchar>(j, i) = a0; // min value, e.g. 0
				else
					if (val >=B)
						NormMImg.at<uchar>(j, i) = a1; // max value, e.g. 230
					else { 
						nval = round(a0 + wsp * (val - A)); 
						if (nval  < a0) nval = a0;
						else if (nval > a1) nval = a1;
						NormMImg.at<uchar>(j, i) = nval;
					}
			}
		}

	}
	// "Rozci¹ganie histogramu" Mono/RGB
	void HistoStretchingRGB(int a0, int A, int B, int a1, ::cv::Mat &MImg, ::cv::Mat &NormMImg,
		::cv::Mat &ColImg, ::cv::Mat &NormColImg)
	{
		int my=MImg.rows;
		int mx=MImg.cols;
		int val, nval, nvalcol;
		double colwsp;
	// RGB image is of type image.at<Vec3b>(y,x)[c]

		NormMImg = MImg.clone();
		NormColImg = ColImg.clone();
		int channels = NormColImg.channels();

		double wsp =  ((double)(a1 - a0)) / ((double)(B - A));

		for (int j=0; j<mx; j++) 
		{ 
			for (int i=0; i<my; i++) 
			{
				val = (int) MImg.at<uchar>(j, i);
				if (val <= A) {
					NormMImg.at<uchar>(j, i) = a0; // min value, e.g. 0
					for (int c = 0; c<3; c++)
						NormColImg.at<::cv::Vec3b>(j, i)[c] = a0;
				}
				else
					if (val >=B)
					{ 
						NormMImg.at<uchar>(j, i) = a1; // max value, e.g. 230
						colwsp = a1/((double)val);
						for (int c = 0; c<3; c++) 
						{	
							nvalcol = (int) (NormColImg.at<::cv::Vec3b>(j, i)[c] * colwsp);
							if (nvalcol > 255)
								nvalcol = 255;
							if (nvalcol < 0) nvalcol = 0;
							NormColImg.at<::cv::Vec3b>(j, i)[c] = nvalcol;
						}
						
					}
					else { 
						nval = round(a0 + wsp * (val - A)); 
						if (nval  < a0) nval = a0;
						else if (nval > a1) nval = a1;
						NormMImg.at<uchar>(j, i) = nval;

						colwsp = nval/((double)val);
						for (int c = 0; c<3; c++) 
						{	
							nvalcol = (int) (NormColImg.at<::cv::Vec3b>(j, i)[c] * colwsp);
							if (nvalcol > 255)
								nvalcol = 255;
							if (nvalcol < 0) nvalcol = 0;
							NormColImg.at<::cv::Vec3b>(j, i)[c] = nvalcol;
						}
						
					}
			}
			//k = k+3;
		}

	}
	// Funkcja pomocnicza w wizualizacji - rysowanie prostok¹ta w obrazie mono
	void DrawBox(::cv::Mat &InImg, int colMin, int colMax, int rowMin, int rowMax, uchar box_col)
	{
		for( int y = rowMin; y < rowMax; y++ )
		{
			InImg.at<uchar>(y, colMin) = box_col;
			InImg.at<uchar>(y, colMax) = box_col;
		}

		for( int x = colMin; x < colMax; x++ )
		{
			InImg.at<uchar>(rowMin, x) = box_col;
			InImg.at<uchar>(rowMax, x) = box_col;
		}
		
	} // Koniec DrawBox

	// Funkcja pomocnicza - wykreœl prost¹ przechodz¹c¹ przez punkt (Cx, Cy) i 
	// zorientowan¹ pod katem alfa (w radianach) - w obrazie mono
	void DrawLine(double alfa, uchar value, int Cx, int Cy, ::cv::Mat &FCImg)
	{
		int my = FCImg.rows;
		int mx = FCImg.cols;
		double tgalfa = tan(alfa);
		int xr, yr;
		double x, y;
		
		// Generuj wsp. y odpowiadaj¹ce kolejnym indeksom x
		for (int i=0; i< mx; i++) {
			y = - (tgalfa * (i - Cx) - Cy); // wspó³rzêdne obrazu, ale zamieñ znak Y 
			yr = round(y); 
			if ((yr >= 0) && (yr < my)) {
				FCImg.at<uchar>(yr, i) = value;
			}
		}
		// Generuj wsp. x odpowiadaj¹ce kolejnym indeksom y 
		for (int i=0; i< my; i++) {
			x = (-(i - Cy))/tgalfa + Cx; // wspó³rzêdne obrazu, ale zamieñ znak Y 
			xr = round(x); 
			if ((xr >= 0) && (xr < mx)) {
				FCImg.at<uchar>(i, xr) = value;
			}
		}
	}

	////////////
	// Funkcja pomocnicza - wykreœla okr¹g w obrazie mono
	void DrawCircle(::cv::Mat &MImg, int radius, int cx, int cy, int dirs, int val ) {		
		double delrad = 2.0 * PI / dirs;
		double alfa = -PI;
		double sinalfa, cosalfa, x, y;
		int x0, y0;
		int dirs2 = dirs/2; 
		int my = MImg.rows;
		int mx = MImg.cols;

		for (int i=0; i< dirs2; i++)
		{ 
			sinalfa = sin(alfa);
			cosalfa = cos(alfa);
			
			x = cx + radius * cosalfa;
			y = -(radius * sinalfa- cy);
			y0 = round(y);
			x0 = round(x);
			if ((x0 >= 0) &&  (x0 < mx)) 
				if ((y0>= 0) &&  (y0 < my)) 
					MImg.at<uchar>(y0,x0) = val;

			x = cx - radius * cosalfa;
			y = radius * sinalfa + cy;
			y0 = round(y);
			x0 = round(x);
			if ((x0 >= 0) &&  (x0 < mx)) 
				if ((y0 >= 0) &&  (y0 < my)) 
					MImg.at<uchar>(y0,x0) = val;
			
			alfa += delrad;
		}
	}

	///////////
	// Funkcja pomocnicza - tworzy obraz 2D histogramu dla funkcji 1D
	void DrawFunct( ::cv::Mat &histoIR, int histSize, ::cv::Mat &histImg) {				 	
		::cv::normalize(histoIR, histoIR, 0, histImg.rows, CV_MINMAX, CV_32F);
		histImg = ::cv::Scalar::all(255);
		int binW = cvRound((double)histImg.cols/histSize);
		for( int i = 0; i < histSize; i++ ) {
			rectangle( histImg, ::cv::Point(i*binW, histImg.rows), 
					 ::cv::Point((i+1)*binW, histImg.rows - cvRound(histoIR.at<float>(i))),
					 ::cv::Scalar::all(0), -1, 8, 0 );
		}
	}
				
	/////////////////////////////////////////
	// G³ówna klasa programu ROD
	//
	class Analysis{
	public:
		// obiekt pomocniczy
		::Calibrator *sfs3D; // wskaŸnik do obiektu pomocniczego

	private:
		// pola : dane we/wy i parametry analizy
		::cv::Mat  *streamIN;
        Properties prop;
        Parameters param;
        UserParameters userparam;
        ResultOD *streamOUT;

		// pola: prze³¹czniki wizualizacji i monitorowania
		int visualSwitch; // 1: czy ma prezentowaï¿½ okna on-line
		int monitorSwitch; // 1: czy ma siï¿½ zatrzymywaï¿½ krok-po-kroku
		int logSwitch; // 1: czy zapisywac wyniki do plikï¿½w 
	
	public:
		void OcenaDefektow(char* dirName, char* outDir, const ResultODline *storedRes)
		{
			//  OcenaDefektow - zasadnicza funkcja detekcji uszkodzeï¿½ w obrazie jabï¿½ka 
            //
            // Dla kaï¿½dej linii i kaï¿½dego widoku jabï¿½ka wywoï¿½uje funkcjï¿½ AnalizaJablka()
            //
            //
            // W wersji off-line funkcji - obrazy sï¿½ podane w plikach graficznych w katalogu 
                           
			int numV = prop.getNumViews();
			int numL = prop.getNumLines();

			//TEST
			cout<< numV << numL << endl;
			cv::waitKey(0);
			//


			char fName[128];
			char outName[128];

            // G³ówna pêtla: dla ka¿dej linii i ka¿dego widoku 
			ResultOD jedenWynik;
            
			for (int i=0; i<numL; i++) //domyœlnie s¹ 3 linie taœmoci¹gu agenta OD
				for (int j=0; j<numV; j++) // domyœlnie jest 8 widoków jab³ka
				{
					// dla wersji off-line:
					// okreœlamy nazwy obrazów w plikach
                    sprintf(fName, "%02d/A/%02d", i+1, j+1); // œcie¿ka i pocz¹tek nazwy plików wejœciowych 
                    sprintf(outName, "%02d_A_%02d", i+1, j+1); // % element nazwy plików z wynikami

					// dla wersji on-line
					// ...

					int offset = i * numL + j;
					
                    // Wywo³anie funkcji AnalizaJablka() dla jednego widoku jednego jab³ka
                    jedenWynik = AnalizaJablka(dirName, fName, outDir, outName, offset);
					
					storedRes[j].resultL[i] = jedenWynik;

					if (this->monitorSwitch == 1) // 1: czy ma siê zatrzymywaæ krok-po-kroku
					{
						cout << "Wykonano analizê: linia " << i << "; widok " << j<< endl;
						cv::waitKey(0);
					}

				}
		}
               


        void DefektyJablka(ResultODline *resultsAll, ResultOD *resultsLine)
		{
         //      % integracja wynikï¿½w dla np. 8 widokï¿½w jabï¿½ka
         //      % we: wyniki z np. 3 linii po np. 8 widokï¿½w
         //      % wy: zintegrowany wynik dla npo. 3 linii
           
		}

			/*
               %alokuj strukturï¿½ wyniku
               resultsL = ROD.ResultODline.empty(pr.numLines, 0); %
               %for i=1: obj.pr.numLines  % dla kaï¿½dej linii w jednym widoku
               %    resultsL(i).result = ROD.ResultOD();
               %end
               
               % TODO: na razie brak kodu
               % proste rozwiï¿½zanie: weï¿½ najgorszy (widok) wynik dla kaï¿½dej linii
               
               % WRAPPER
               for i=1: obj.pr.numLines
                   resultsL(i).result = resultsAll(obj.pr.numViews).resultL(i).result;
               end
            end
           */
		
        
		// Konstruktor
        Analysis(cv::Mat *in, Properties pr, Parameters pa, UserParameters upa, ResultOD *ou,
			int vis=0, int monit=0, int log=0)
		{
			this->streamIN = in;
            this->prop = pr;
            this->param = pa;
            this->userparam = upa;
            this->streamOUT = ou;
			
			this->visualSwitch = vis; // 1: czy ma prezentowaæ okna on-line; musi byæ w³¹czony tryb monitorowania 
			this->monitorSwitch = monit; // 1: czy ma drukowaæ komentarze wykonania kroków 
			this->logSwitch = log; // 1: czy ma utrwalaæ okna z wynikami w postaci plików; aktywny w trybie monitorowania
		}
		
		Analysis()
		{
		}

            
        /**
        * @fn Analiza pojedynczego widoku jabï¿½ka w celu wykrycia defektï¿½w na powierzchni jabï¿½ka
        
        // PARAMETRY FUNKCJI
        //  fileName - podstawowa nazwa pliku graficznego png;
        //  dirName - nazwa katalogu
        //  outDir - katalog dla wynikï¿½w
		//  wynik - wynik - struktura typu ResultOD
		// WYNIK
		// wynik - opis pojedynczego widoku jednego jabï¿½ka typu ResultOD
        // CEL:
        // Analiza ksztaï¿½tu 2D i powierzchni 3D w obrazie IR
        // oraz analiza ksztaï¿½tu 2D i kolorï¿½w obszarï¿½w w obrazie RGB/Mono
        //
        // Parametry wejsciowe: nazwy plikï¿½w (docelowo strumieï¿½ wejï¿½ciowy) i
        // katalog na wyniki
        // Wynik: struktura typu ResultOD
        //
        // Wywo³uje funkcje (opcje):
        
		// SFS: AnalizaSFS3D() // (K.Przerwa)
		// vector<StatsRoi> rois = solution->solution("01_0.png");
            
		// STEM:
		// OdstajacaSzypulka(), SzypulkaWObrysie() // (A. Wilkowski)
        
		// i szereg funkcji pomocniczych    
        // 
        // Autor: Wlodzimierz Kasprzak (wsp. Jan Figat)          
        // Data: 10-10-2013
        // Modyfikacja: 11-12-2014
        // 
		*/

        ResultOD AnalizaJablka(char* dirName, char* fileName, char* ouDir, char* ouName, int offset, cv::Mat in_ir_img = cv::Mat(), cv::Mat in_rgb_img = cv::Mat())
		{
			// Ustaw obiekty - PARAMETRY analizy obrazu
			Parameters param = this->param;
            Properties prop = this->prop;
            UserParameters uparam = this->userparam;

			// Prze³¹czniki - wizualizacja i monitorowanie wyników czêœciowych
			int VISUAL = this->visualSwitch; // 1: czy ma prezentowaï¿½ okna on-line
			int MONITOR = this->monitorSwitch; // 1: czy ma siï¿½ zatrzymywaï¿½ krok-po-kroku
			int LOGS = this->logSwitch; // 1: utrwalenie wynikï¿½w w plikach dla ilustracji dziaï¿½ania funkcji
			
			// Opcje wykonania kodu
			int STEMsw = prop.getUseSTEM(); // analiza wewnêtrznych szypu³ek
			int SFSsw = prop.getUseSFS(); // analiza 3D obrazu IR

			// Parametry szczegó³owe
			int DIRS = param.directions; // 720: podzia³ k¹ta pe³nego na 720 czêœci
			

			// Nazwy plików
			char fIrName[128], fColName[128];
			char outName[128];
			// Pomiar czasu
			double t0, elapsed;
			time_t start, end;
			// Zakres k¹towy przeszukiwania konturu i wnêtrza
			cv::Mat wycinekPoIr = cv::Mat::zeros(2,1,CV_32SC1);
			cv::Mat wycinek = cv::Mat::zeros(3,1,CV_32SC1);
			double DWAPI;
			

			// Zaczynamy
			
			// TEST
			//	ouDir="D:/workspace/SORTER_Analysis/img/Database/results/log2";
			// 

			if (MONITOR == 1) std::cout<< "AnalizaJablka() start\n"; 
			

			::cv::Mat IRImg;
			::cv::Mat ColorImg;
			if (dirName) {
				// 1.1 obraz IR 
				//
				sprintf(fIrName, "%s/%s_0.png", dirName, fileName); 
				
				if (MONITOR == 1) std::cout<< fIrName << std::endl; 
				
				IRImg =  ::cv::imread(fIrName);
				if(! IRImg.data ){									///sprawdzenie czy plik istnieje
						cout<<"\n"<<fIrName<<" -- IR"<<endl;
								throw  "Nie mo¿na otworzyæ pliku" ;
				}
				if (MONITOR == 1) std::cout << "L kana³ow IRImg: " << IRImg.channels() << std::endl; 
				if (VISUAL== 1) { 	::cv::imshow("A1.1 Obraz IR", IRImg); }

				// 1.2 obraz kolorowy
				sprintf(fColName, "%s/%s_1.png", dirName, fileName); 
				if (MONITOR == 1) std::cout<< fColName << std::endl; 
				ColorImg =  ::cv::imread(fColName);
				if(! ColorImg.data ){								///sprawdzenie czy plik istnieje
						cout<<"\n"<<fColName<<" -- RGB"<<endl;
					throw  "Nie mo¿na otworzyæ pliku" ;			
				}
				if (MONITOR == 1) std::cout<< "L kana³ow ColorImg: " << ColorImg.channels() << std::endl; 

				if (VISUAL== 1) { ::cv::imshow("A1.2 Obraz RGB", ColorImg); }
			} else {
				IRImg = in_ir_img;
				ColorImg = in_rgb_img;
			}
			
			//1.3 Utwórz obraz mono z obrazu kolorowego RGB
			::cv::Mat MonoImg;
			::cv::cvtColor(ColorImg, MonoImg, CV_BGR2GRAY);
			if (VISUAL== 1) { ::cv::imshow("A1.3 Obraz Mono", MonoImg); }
			if (MONITOR == 1) 
				std::cout<< "L kana³ow MonoImg: " << MonoImg.channels() << std::endl;
			
			int sy = MonoImg.rows;
			int sx = MonoImg.cols;
			int gy = IRImg.rows;
			int gx = IRImg.cols;
			int cy = sy;
			int cx = sx;
			//
			vector<cv::Mat> IRplanes; // Vector to klasa szablonowa
			::cv::split(IRImg, IRplanes); // Rozdziel obraz na 3 p³aty 

			//
			//1.4. Normalizacja histogramu - rozci¹ganie (by³o), przeskalowanie (jest)
			//
			int histSize = 256;
			::cv::Mat histoMono, histoIR;
			::cv::Mat histImage = ::cv::Mat::ones(256, 256, CV_8U)*255;
			//1.4a,b Wyznacz histogramy obrazu mono i IR (tylko w trybie wizualizacji)
			if (VISUAL== 1) { 
				::cv::calcHist(&MonoImg, 1, 0, ::cv::Mat(), histoMono, 1, &histSize, 0);
				
				 ::cv::normalize(histoMono, histoMono, 0, histImage.rows, CV_MINMAX, CV_32F);
				 histImage = ::cv::Scalar::all(255);
				 int binW = cvRound((double)histImage.cols/histSize);
				 for( int i = 0; i < histSize; i++ ) 
					 rectangle( histImage, ::cv::Point(i*binW, histImage.rows), 
					 ::cv::Point((i+1)*binW, histImage.rows - cvRound(histoMono.at<float>(i))),
					 ::cv::Scalar::all(0), -1, 8, 0 ); 
				 ::cv::imshow("A1.4a Histogram obrazu Mono", histImage); 
			}
			if (VISUAL== 1) {
				 ::cv::calcHist(&IRplanes[0], 1, 0, ::cv::Mat(), histoIR, 1, &histSize, 0);
				 
				 ::cv::normalize(histoIR, histoIR, 0, histImage.rows, CV_MINMAX, CV_32F);
				 histImage = ::cv::Scalar::all(255);
				 int binW = cvRound((double)histImage.cols/histSize);
				 for( int i = 0; i < histSize; i++ ) 
					 rectangle( histImage, ::cv::Point(i*binW, histImage.rows), 
					 ::cv::Point((i+1)*binW, histImage.rows - cvRound(histoIR.at<float>(i))),
					 ::cv::Scalar::all(0), -1, 8, 0 ); 
				::cv::imshow("A1.4b Histogram obrazu IR", histImage); 
			}

			// 1.5 Normalizacja histogramu
			// W oryginale: by³o rozci¹ganie histogramu obrazu mono i RGB
			// z parametrami [0.05 0.95]  i odwzorowaniem [0.0 0.90]*255
			
			double minVal, maxVal;
			::cv::minMaxIdx(MonoImg, &minVal, &maxVal, NULL, NULL, ::cv::noArray());
			::cv::Mat NormMonoImg;
			::cv::Mat NormColorImg;
			
			double alpha = 230.0/ maxVal; // = 0.9 * 255 to maksymalna wartoœæ
			// 1.5.a Rozci¹ganie histogramu - zbli¿one do oryginalnego rozwi¹zania
			// jedynie próg dolny (20) jest szacowany
			alpha = 0.6 * maxVal;
			cout<< "MaxVal for Mono/RGB: "<< maxVal << ", alpha: " << alpha << endl;
			if (alpha > 200)
				alpha = 200;
			ROD::HistoStretchingRGB(0, 10, (int)alpha, 230, MonoImg, NormMonoImg, ColorImg, NormColorImg);
			// alternatywnie:
			// ze wzgl¹du na brak operacji "rozci¹gania histogramu" w opencv
			// zastosujemy zwyk³e przeskalowanie wartoœci do zakresu [0, 0.9 * 255]
			// MonoImg.convertTo(NormMonoImg, CV_8U, alpha);
			//alpha = 230.0/ maxVal; // = 0.9 * 255 to maksymalna wartoœæ
			//ColorImg.convertTo(NormColorImg, -1, alpha);
			

			// 1.5.b Podobnie rozci¹ganie/przeskalowanie dla obrazu IR - 
			// alternatywnie przeskalowanie zamiast rozci¹gania histogramu
			::cv::Mat NormIRImg;
			::cv::minMaxIdx(IRplanes[0], &minVal, &maxVal, NULL, NULL, ::cv::noArray());
			alpha = 230.0/ maxVal;
			alpha = 0.9 * maxVal;
			cout<< "MaxVal for IR: "<< maxVal << ", alpha: " << alpha << endl;

			if (alpha > 200)
				alpha = 200;
			// Rozci¹ganie
			ROD::HistoStretching(0, 20, (int)alpha, 230, IRplanes[0], NormIRImg);
			// O ile IRImg ma 3 p³aty, to NormIRImg bedzie mia³ ju¿ tylko 1
			// lub przeskalowanie
			// IRplanes[0].convertTo(NormIRImg, CV_8U, alpha);
			
			// Wizualizacja wyniku
			if (VISUAL== 1) { 
				::cv::imshow("1.5a Norm mono", NormMonoImg); 
				::cv::imshow("1.5b Norm kolor", NormColorImg); 
				::cv::imshow("1.5c Norm IR", NormIRImg); 
			}
			// Utrwalenie wyniku
			if (LOGS == 1)
			{ 
				sprintf(outName, "%s/A1_5_NormMono_%s.png", ouDir, ouName); 
				cv::imwrite(outName, NormMonoImg);
				sprintf(outName, "%s/A1_5_NormColor_%s.png", ouDir, ouName); 
				cv::imwrite(outName, NormColorImg);
				sprintf(outName, "%s/A1_5_NormIR_%s.png", ouDir, ouName); 
				cv::imwrite(outName, NormIRImg);
			}


			//1.6 Wyznacz histogramy po przeskalowaniu (tylko w trybie wizualizacji)
			if (VISUAL== 1) { 
				 ::cv::calcHist(&NormMonoImg, 1, 0, ::cv::Mat(), histoMono, 1, &histSize, 0);
				 // wyzeruj dla wartoœci 0
				 histoMono.at<float>(0,0) = 0;
				 
				 ::cv::normalize(histoMono, histoMono, 0, histImage.rows, CV_MINMAX, CV_32F);
				 histImage = ::cv::Scalar::all(255);
				 int binW = cvRound((double)histImage.cols/histSize);
				 for( int i = 0; i < histSize; i++ ) 
					 rectangle( histImage, ::cv::Point(i*binW, histImage.rows), 
					 ::cv::Point((i+1)*binW, histImage.rows - cvRound(histoMono.at<float>(i))),
					 ::cv::Scalar::all(0), -1, 8, 0 ); 
				 ::cv::imshow("1.6a Histogram Mono po normalizacji", histImage); 
			}
			if (VISUAL== 1) { 
				 ::cv::calcHist(&NormIRImg, 1, 0, ::cv::Mat(), histoIR, 1, &histSize, 0);
				 // wyzeruj dla wartoœci 0
				 histoIR.at<float>(0,0) = 0;

				 ::cv::normalize(histoIR, histoIR, 0, histImage.rows, CV_MINMAX, CV_32F);
				 histImage = ::cv::Scalar::all(255);
				 int binW = cvRound((double)histImage.cols/histSize);
				 for( int i = 0; i < histSize; i++ ) 
					 rectangle( histImage, ::cv::Point(i*binW, histImage.rows), 
					 ::cv::Point((i+1)*binW, histImage.rows - cvRound(histoIR.at<float>(i))),
					 ::cv::Scalar::all(0), -1, 8, 0 ); 
				::cv::imshow("1.6b Histogram IR po normalizacji", histImage); 	
				
			}
			// Koniec pomiaru czasu dla kroku A1
			end=time(0);
			elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
			cout << "Czas kroku 1: " << endl;
			cout << elapsed<< " ms" <<  endl;
			cout << end-start << " s" <<  endl;
			// Koniec A1
			////////////////////////////
	
			//cvWaitKey();


			////////////////////////////
			// A2. Obraz krawêdziowy w mono i IR 
			//
			t0=clock();		// zliczanie czasu
			start=time(0);
			// 2.1 Operator krawêdziowy w obrazie Mono
			::cv::Mat EdgeMImg, EdgeDirImg;
			// Canny
			
			//::cv::Canny(NormMonoImg, EdgeMImg, param.edgeThresh *255, param.edgeThresh*2, 3, false);
			
			// Jednak nie mamy kierunku krawêdzi dla elementu krawêdziowego
			// Dlatego potrzebna jest w³asna funkcja:
			// Operator krawêdziowy i pocienianie z progiem wzglêdnym, np. edgeThresh=0.2:
			EdgeDetection(param.lowThresh, param.edgeThresh, NormMonoImg, EdgeMImg, EdgeDirImg);
			
			if (VISUAL== 1) { 
				::cv::Mat ShowImg = ::cv::Mat::zeros(20, 20, CV_8U);
				NormMonoImg.copyTo(ShowImg, EdgeMImg);
				cv::imshow("A2.1 Maska krawêdzi w Mono", ShowImg);
				// Dokumentacja wyniku
				if (LOGS == 1)
				{ 
					sprintf(outName, "%s/A2_1_EdgeMono_%s.png", ouDir, ouName); 
					cv::imwrite(outName, EdgeMImg);
				}
				ShowImg.release();
			}
			
			//
			// 2.2 Operator krawêdziowy w IR
			::cv::Mat EdgeIRMImg, EdgeIRDirImg;
			// Funkcja globalna
			EdgeDetection( param.lowThresh, param.edgeThresh, NormIRImg, EdgeIRMImg, EdgeIRDirImg);
			if (VISUAL== 1) { 
				::cv::Mat ShowImg = ::cv::Mat::zeros(20, 20, CV_8U);
				NormIRImg.copyTo(ShowImg, EdgeIRMImg);
				cv::imshow("A2.2 Maska krawêdzi w IR", ShowImg);
				// Dokumentacja wyniku
				if (LOGS == 1)
				{ 
					sprintf(outName, "%s/A2_2_EdgeIR_%s.png", ouDir, ouName); 
					::cv::imwrite(outName, EdgeIRMImg);
				}
				ShowImg.release();
			}    
			// Koniec pomiaru czasu dla kroku 2
			end=time(0);
			elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
			cout << "Czas kroku 2: " << endl;
			cout << elapsed<< " ms" <<  endl;
			cout << end-start << " s" <<  endl;
			// Koniec A2
			///////////////////////////////


			///////////////////////////////
			// A3. Wyznaczenie prostok¹tnego obszaru zainteresowania ROI
			// 
			
			t0=clock();		// zliczanie czasu
			start=time(0);
			// 3.1 W obrazie mono
			// Prostok¹tny obszar ROI w obrazie mono ograniczony jest wartoœciami:
			// columnMin, columnMax, rowMin, rowMax

			int columnMin, columnMax, rowMin, rowMax;
			int bbox[4]; 
			DetectROI(NormMonoImg, bbox); // funkcja pomocnicza w obszarez ROD 

			columnMin=bbox[0];
			columnMax=bbox[1];
			rowMin=bbox[2];
			rowMax=bbox[3];

			//TEST
			cout<<"bbox mono:" << columnMin << ", "<< columnMax << ", " << rowMin << ", " << rowMax <<endl;
			//

			// Poka¿ obszar ROI w obrazie mono
			if (VISUAL == 1)
			{
				::cv::Mat ShowROI_MonoImg = NormMonoImg.clone(); 
				ROD::DrawBox(ShowROI_MonoImg, columnMin, columnMax, rowMin, rowMax, 255);
				//::cv::Mat ShowROI_MonoImg = MonoImg(::cv::Rect(columnMin, rowMin, columnMax-columnMin+1, rowMax-rowMin+1));
				//std::cout<<"column_min/row_min "<<columnMin<<" / "<<rowMin<<"\n";
				cv::imshow("A3.1 ROI mono", ShowROI_MonoImg);
				// Dokumentacja wyniku
				if (LOGS == 1)
				{ 
					sprintf(outName, "%s/A3_1_ROIMono_%s.png", ouDir, ouName); 
					::cv::imwrite(outName, ShowROI_MonoImg);
				}
				ShowROI_MonoImg.release();
			}

			//3.2 ROI w obrazie IR
			int IRcolumnMin, IRcolumnMax, IRrowMin, IRrowMax;
			DetectROI2(NormIRImg, param.cornerMax, param.cornerMin, bbox);
			IRcolumnMin=bbox[0];
			IRcolumnMax=bbox[1];
			IRrowMin=bbox[2];
			IRrowMax=bbox[3];
			//TEST
			cout<<"IR bbox:" << IRcolumnMin << ", "<< IRcolumnMax << ", " << IRrowMin << ", " << IRrowMax <<endl;
			
			

			// Poka¿ obszar ROI w obrazie IR
			if(VISUAL==1)
			{
				::cv::Mat ShowROI_IRImg = NormIRImg.clone();
				ROD::DrawBox(ShowROI_IRImg, IRcolumnMin, IRcolumnMax, IRrowMin, IRrowMax, 255);
				//::cv::Mat ShowROI_IRImg = IRImg(::cv::Rect(int(IRcolumnMin), int(IRrowMin), int(IRcolumnMax-IRcolumnMin)+1, int(IRrowMax-IRrowMin)+1 ));
				cv::imshow("A3.2 ROI IR", ShowROI_IRImg);
				// Dokumentacja wyniku
				if (LOGS == 1)
				{ 
					sprintf(outName, "%s/A3_2_RoiIR_%s.png", ouDir, ouName); 
					::cv::imwrite(outName, ShowROI_IRImg);
				}
				ShowROI_IRImg.release();
			}
			

			/*
			// 3.3 ? Sprawdzenie detekcji konturu operatorami morfologicznymi 
			::cv::Mat dst2;
			::cv::Rect rect;
			::cv::Mat dst = NormMonoImg.clone();
			floodFill(dst, ::cv::Point(120, 150), ::cv::Scalar(230), &rect, ::cv::Scalar(3), ::cv::Scalar(3));

			//ErosionEllipse( NormMonoImg,  dst);
			// 3.4 ?
			DilationEllipse( dst,  dst2);
			// Wizualizacja wyniku operacji morfologicznych
			if (VISUAL == 1) {
				cv::imshow( "A3.3 Fill", dst );
				cv::imshow( "A3.4 Dilation", dst2 );
			}
			*/

			// Koniec pomiaru czasu dla kroku 3
			end=time(0);
			elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
			cout << "Czas kroku 3: " << endl;
			cout << elapsed<< " ms" <<  endl;
			cout << end-start << " s" <<  endl;
	
			//cv::waitKey();
			
			//
			// A4. ZnajdŸ dok³adny brzeg obiektu (kszta³t) w obrazie kolorowym i IR
			//
			// Kontur w obrazie RGB
			// TO DO : na razie dane globalne
			// ...


			// Kontur w obrazie IR
			int Cirx, Ciry; // œrodek masy konturu w obrazie IR
			::cv::Mat IrKontur = ::cv::Mat::zeros(DIRS, 2, CV_64FC1); // wektor wspó³rzêdnych punktów konturu
			::cv::Mat IrKontur1D = ::cv::Mat::zeros(DIRS, 1, CV_64FC1); // odleg³oœci punktów konturu od œrodka masy
			::cv::Mat IrGrad1D = ::cv::Mat::zeros(DIRS, 1, CV_64FC1); // gradient funkcji odleg³oœci
			//


			t0=clock();             // zliczanie czasu
			start=time(0);

			// 4.1 Szukamy konturu jab³ka w obrazie IR
			   
			int InitCy, InitCx; // œrodek spodziewanego obszaru ROI w obrazie IR
			double dIRcolumnMin, dIRcolumnMax, dIRrowMin, dIRrowMax; 
			int radiusIR; // dla koñcowego promienia konturu
			int minRadius; // ograniczenie w procesie poszukiwania konturu

			// IRrowMin, IRrowMax, IRcolumnMin, IRcolumnMax to spodziewane "pierwsze", przybli¿one ROI
			dIRcolumnMin = IRcolumnMin;
			dIRcolumnMax = IRcolumnMax;
			dIRrowMin = IRrowMin;
			dIRrowMax = IRrowMax;
			InitCy = round((dIRrowMin + dIRrowMax)/2);
			InitCx = round((dIRcolumnMin + dIRcolumnMax)/2);
			minRadius = round(((dIRcolumnMax - dIRcolumnMin) + (dIRrowMax - dIRrowMin))/ 4.0);
			
			// Jednak szukamy najpierw w pe³nym obrazie IR:
			// Funkcja szukania konturu w obrazie IR
			// Wynik zwracany jest w postaci zmiennych globalnych (trzeba zmieniæ):
			// Cirx, Ciry, IrKontur, IrKont1D, GradIrKont1D
			// oryg. 0.2, 0.3
			KonturIR(0.15, 0.20, DIRS, InitCx, InitCy, minRadius, NormIRImg, 
				 Cirx, Ciry, IrKontur, IrKontur1D,  IrGrad1D);
			
			// Teraz w³aœciwy obszar ROI odpowiada ograniczeniom konturu IR:
			// - dodajemy te¿ niewielki margines
			cv::minMaxLoc(IrKontur.col(0), &dIRcolumnMin, &dIRcolumnMax,0,0,cv::Mat());
			
			if (dIRcolumnMin <0)	dIRcolumnMin =0;
			if (dIRcolumnMax >= gx) dIRcolumnMax = gx-1;
			
			cv::minMaxLoc(IrKontur.col(1),&dIRrowMin,&dIRrowMax,0,0,cv::Mat());
			
			if (dIRrowMin <0)  dIRrowMin =0;
			if (dIRrowMax >= gy)  dIRrowMax = gy-1;
			
			// Zamiana bbox na wartoœci ca³kowite
			IRcolumnMin = int(dIRcolumnMin);
			IRcolumnMax = int(dIRcolumnMax);
			IRrowMin = int(dIRrowMin);
			IRrowMax = int(dIRrowMax);

			// Wizualizacja konturu w IR
			// w kolejnym kroku 5.1 

			// Koniec pomiaru czasu dla kroku 4
			end=time(0);
			elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
			cout << "Czas kroku 4: " << endl;
			cout << elapsed<< " ms" <<  endl;
			cout << end-start << " s" <<  endl;


			///////////////
			// 5.-7. Detekcja szypu³ki : odstaj¹cej i wewnêtrznej
			// - odstaj¹cej : kroki 5.1 i 5.2 (RodFunctions): obraz mono analiza 1-wym. funkcji konturu, 
			// lub (alternatywnie) : krok 6 (obszar STEM) (AW): obraz IR: analiza 2D
			// - wewnêtrznej: krok 7 (obszar STEM)(AW): obraz RGB/Mono: analiza 2D 
			// lub (alternatywnie do 7: krok 5.3 (RodFunctions) - nieuruchomiony w kodzie C++)
			//

			t0=clock();         // pocz¹tek zliczania czasu
			start=time(0);

			// 5.1.a) Analiza funkcji 1D konturu (WKas) w obrazie IR
			cv::Mat Wycinek = cv::Mat::zeros(3,1,CV_32SC1);
			// Szukamy w ca³ym zakresie konturu
			Wycinek.at<int>(0,0)= 0; 
			Wycinek.at<int>(1,0)= DIRS-1;
			Wycinek.at<int>(2,0)= DIRS;

			// Funkcja zwraca wynik w postaci zmiennych globalnych:
			// [jestOgonIr, pozIrOgon, alfaIr, CirxNew, CiryNew, MaskaIrOgon, BBoxIr]
			
			int CirxN, CiryN, jestOgonIr0;

	//Zmieniona wartoœc 120.0 (wspPIK)
			::cv::Mat BBoxIr = OgonekIR(gy, gx, /*120.0*//*10.0*/5, Cirx, Ciry, Wycinek, IrKontur, IrKontur1D, IrGrad1D,
				CirxN, CiryN);

			// Po ewentualnym uwzglêdnieniu szypu³ki odstaj¹cej
			// zmieni³ siê BBox obszaru samego jablka
				IRcolumnMin = BBoxIr.at<int>(0,0); // xmin
				IRcolumnMax = BBoxIr.at<int>(2,0); // xmax
				IRrowMin = BBoxIr.at<int>(1,0); // ymin
				IRrowMax = BBoxIr.at<int>(3,0); // ymax
			// Promieñ konturu:
				radiusIR = BBoxIr.at<int>(4,0); // promieñ
			// Zapmiêtaj istnienie szypu³ki zewnêtrznej w obrazie IR
				jestOgonIr0 = jestOgonIr;

			// Utwórz roi szypu³ki w IR
			cv::Mat roiSzypIr = cv::Mat::zeros(6,1,CV_32SC1);
			if (jestOgonIr == 1)
			{
				roiSzypIr.at<int>(0,0) =(pozIrOgon.at<int>(0,0) - 20); // xmin
				if (roiSzypIr.at<int>(0,0) < 0) 
					roiSzypIr.at<int>(0,0) = 0;
				roiSzypIr.at<int>(1,0) =(pozIrOgon.at<int>(1,0) - 20); // ymin
				if (roiSzypIr.at<int>(1,0) < 0) 
					roiSzypIr.at<int>(1,0) = 0;
				roiSzypIr.at<int>(2,0) =(pozIrOgon.at<int>(0,0) + 20); // xmax
				if (roiSzypIr.at<int>(2,0) >= gx) 
					roiSzypIr.at<int>(2,0) = gx-1;
				roiSzypIr.at<int>(3,0) =(pozIrOgon.at<int>(1,0) + 20); // ymax
				if (roiSzypIr.at<int>(3,0) >= gy) 
					roiSzypIr.at<int>(3,0) = gy-1;
				roiSzypIr.at<int>(4,0) = pozIrOgon.at<int>(0,0); // xc ogonka
				if (roiSzypIr.at<int>(4,0) >= gx) 	roiSzypIr.at<int>(4,0) = gx-1;
				roiSzypIr.at<int>(5,0) = pozIrOgon.at<int>(1,0); // yc ogonka
				if (roiSzypIr.at<int>(5,0) >= gy) 	roiSzypIr.at<int>(5,0) = gy-1;
				
				if (MONITOR == 1)
					cout<<"IrOgon: x="<< pozIrOgon.at<int>(0,0) <<", y=" << pozIrOgon.at<int>(1,0) << endl;
				
			}
			else
			{
				roiSzypIr.at<int>(0,0) = 0;
				roiSzypIr.at<int>(1,0) = 0;
				roiSzypIr.at<int>(2,0) = 0;
				roiSzypIr.at<int>(3,0) = 0;
				roiSzypIr.at<int>(4,0) = 0;
				roiSzypIr.at<int>(5,0) = 0;
			}

			// Wizualizacja konturu i ogonka w obrazie IR
			cv::Mat MonoE3Img;
			cv::Mat ShowMonoE3Img;
			std::vector<cv::Mat> MonoE3Img_planes(3);
			if (VISUAL == 1)
			{
				// Wizualizacja konturu w obrazie
				int x,y;
				//cv::split(F1ColorImg,MonoE3Img_planes);

				MonoE3Img_planes[0] = NormIRImg.clone();
				MonoE3Img_planes[1] = NormIRImg.clone();
				MonoE3Img_planes[2] = NormIRImg.clone();
				
			    for (int i=0;i<DIRS;i++)
			    {
			        y = int(IrKontur.at<double>(i,1) );
			        x = int(IrKontur.at<double>(i,0) );

			        if (MaskaIrOgon.at<uchar>(i,0) == 1)
			        {
			        	// ogonek
			        	MonoE3Img_planes[0].at<uchar>(y,x) = 255;
			        	MonoE3Img_planes[1].at<uchar>(y,x) = 255;
			        	MonoE3Img_planes[2].at<uchar>(y,x) = 255;
			        }
			        else
			        {
			            // kontur
			        	MonoE3Img_planes[0].at<uchar>(y,x) = 255;
			        	MonoE3Img_planes[1].at<uchar>(y,x) = 100;
			        	MonoE3Img_planes[2].at<uchar>(y,x) = 0;
			        }
			    }
				// Ewentualnie dorysuj ROI ogonka
				if (jestOgonIr == 1) {
					ROD::DrawBox(MonoE3Img_planes[0], roiSzypIr.at<int>(0,0), roiSzypIr.at<int>(2,0),roiSzypIr.at<int>(1,0),roiSzypIr.at<int>(3,0),100);
					ROD::DrawBox(MonoE3Img_planes[1], roiSzypIr.at<int>(0,0), roiSzypIr.at<int>(2,0),roiSzypIr.at<int>(1,0),roiSzypIr.at<int>(3,0),100);
					ROD::DrawBox(MonoE3Img_planes[2], roiSzypIr.at<int>(0,0), roiSzypIr.at<int>(2,0),roiSzypIr.at<int>(1,0),roiSzypIr.at<int>(3,0),255);

				// Dorysuj oœ
					DrawLine(alfaIr.at<double>(0,0), 255, CirxN, CiryN, MonoE3Img_planes[0]); // red plane
					DrawLine(alfaIr.at<double>(1,0), 255, CirxN, CiryN, MonoE3Img_planes[0]); // red plane
					DrawLine(alfaIr.at<double>(0,0), 150, CirxN, CiryN, MonoE3Img_planes[1]); // g plane
					DrawLine(alfaIr.at<double>(1,0), 150, CirxN, CiryN, MonoE3Img_planes[1]); // g plane
					DrawLine(alfaIr.at<double>(0,0), 50, CirxN, CiryN, MonoE3Img_planes[2]); // b plane
					DrawLine(alfaIr.at<double>(1,0), 50, CirxN, CiryN, MonoE3Img_planes[2]); // b plane
				}

				// Narysuj prostok¹t obejmuj¹cy obszar jab³ka (bez szypu³ki) 
				ROD::DrawBox(MonoE3Img_planes[0], IRcolumnMin, IRcolumnMax, IRrowMin, IRrowMax, 0);
				ROD::DrawBox(MonoE3Img_planes[0], IRcolumnMin, IRcolumnMax, IRrowMin, IRrowMax, 100);
				ROD::DrawBox(MonoE3Img_planes[0], IRcolumnMin, IRcolumnMax, IRrowMin, IRrowMax, 255);

				// Dorysuj œrodek
				ROD::DrawBox(MonoE3Img_planes[0], CirxN - 2, CirxN + 2, CiryN - 2, CiryN + 2, 255);
				ROD::DrawBox(MonoE3Img_planes[1], CirxN - 2, CirxN + 2, CiryN - 2, CiryN + 2, 0);
				ROD::DrawBox(MonoE3Img_planes[2], CirxN - 2, CirxN + 2, CiryN - 2, CiryN + 2, 0);

				// Dorysuj okr¹g aproksymuj¹cy kontur jab³ka
				cout<< "radiusIR: " << radiusIR << endl;

				ROD::DrawCircle(MonoE3Img_planes[2], radiusIR, CirxN, CiryN, DIRS, 250) ;		
				ROD::DrawCircle(MonoE3Img_planes[1], radiusIR, CirxN, CiryN, DIRS, 100) ;
				//
			    cv::merge(MonoE3Img_planes, ShowMonoE3Img /*MonoE3Img*/);
			    cv::imshow("A5.1 Kontur w IR", ShowMonoE3Img /*MonoE3Img*/);
			    if (LOGS == 1)
			    {
			    	sprintf(outName, "%s/A5_1_KonturIRSzyp_%s.png", ouDir, ouName);
			    	cv::imwrite(outName, ShowMonoE3Img /*MonoE3Img*/);
			    }
			    
				ShowMonoE3Img.release();
				MonoE3Img_planes[0].release();
				MonoE3Img_planes[1].release();
				MonoE3Img_planes[2].release();

				// Wizualizacja funkcji 1D Konturu
				cv::Mat histImg = cv::Mat::zeros(400,400,CV_32SC1);
				// TO DO : poprawiæ funkcjê
				// DrawFunct(IrKontur1D, DIRS, histImg);
				//cv::imshow("IR Kontur 1D", histImg);			
				histImg.release();

			}

			// 5.2 Detekcja konturu w obrazie RGB			
			::cv::Mat Kontur = ::cv::Mat::zeros(720, 2, CV_64FC1); // element co pewien stopieñ k¹towy
			::cv::Mat Kontur1D = ::cv::Mat::zeros(720, 1, CV_64FC1);
			::cv::Mat Grad1D = ::cv::Mat::zeros(720, 1, CV_64FC1);
			int radiusMono;

			// 5.2a Przytnij obraz kolorowy do obszaru ROI
			//::cv::Mat F1ColorImg = NormColorImg(::cv::Rect(columnMin, rowMin, columnMax- columnMin+1,  rowMax- rowMin+1)).clone();
			//::cv::Mat F1MonoImg = NormMonoImg(::cv::Rect(columnMin, rowMin, columnMax- columnMin+1,  rowMax- rowMin+1)).clone();
			//::cv::Mat F1EdgeMImg = EdgeMImg(::cv::Rect(columnMin, rowMin, columnMax- columnMin+1,  rowMax- rowMin+1)).clone();
			//::cv::Mat F1EdgeDirImg = EdgeDirImg(::cv::Rect(columnMin, rowMin, columnMax- columnMin+1,  rowMax- rowMin+1)).clone();
			// lub 5.2a : bez przycinania:
			::cv::Mat F1ColorImg, F1MonoImg, F1EdgeMImg, F1EdgeDirImg ;
			NormColorImg.copyTo(F1ColorImg);
			NormMonoImg.copyTo(F1MonoImg);
			EdgeMImg.copyTo(F1EdgeMImg);
			EdgeDirImg.copyTo(F1EdgeDirImg);

			int cyf1 = F1ColorImg.rows;
			int cxf1 = F1ColorImg.cols;
			int Cx, Cy; // œrodek konturu w obrazie przyciêtym RGB/Mono
			// po przyciêciu
			//Cx = round((columnMax - columnMin) * 0.5);
			//Cy = round((rowMax - rowMin) * 0.5);
			// lub bez przyciêcia
			Cx = round(columnMin + (columnMax - columnMin) * 0.5);
			Cy = round(rowMin + (rowMax - rowMin) * 0.5);

			// 5.2b Szukamy konturu jab³ka w obrazie RGB (ewentualnie ograniczonym do ROI)
			KonturRGB(param.intensityThresh, param.redColorThresh, DIRS, Cx, Cy, radiusIR, F1ColorImg, F1MonoImg,
				 Kontur, Kontur1D, Grad1D); // zwracany wynik
			// Dla ograniczenia obszaru przeszukiwania za³o¿ono ten sam promieñ rzeczywisty co w obrazie IR
			
			// Teraz mo¿na wyznaczyc œredni promieñ w obrazie RGB/Mono
			//double sumRad = 0.0;
			//for (int i=0; i<DIRS; i++)
			//	sumRad += Kontur1D.at<double>(i,0);
			//radiusMono = round(sumRad/DIRS);
			radiusMono = round(cv::mean(Kontur1D).val[0]);

			// Wizualizacja konturu w obrazie RGB
			if (VISUAL == 1)
			{
			    int x, y;//
			    cv::Mat Show3F1MonoImg;
			    Show3F1MonoImg = F1ColorImg.clone();//F1MonoImg;//F1ColorImg;
			    std::vector<cv::Mat> Show3F1MonoImg_planes(3);
			    cv::split(Show3F1MonoImg, Show3F1MonoImg_planes);

			    for (int i=0;i<DIRS;i++)
			      {
			        y = int( Kontur.at<double>(i,1) );
			        x = int( Kontur.at<double>(i,0) );
			        Show3F1MonoImg_planes[0].at<uchar>(y,x) = 255;
			        Show3F1MonoImg_planes[1].at<uchar>(y,x) = 120;
			        Show3F1MonoImg_planes[2].at<uchar>(y,x) = 0;  
			      }
				// Dorysuj œrodek
				ROD::DrawBox(Show3F1MonoImg_planes[0], Cx - 2, Cx + 2, Cy - 2, Cy + 2, 255);
				ROD::DrawBox(Show3F1MonoImg_planes[1], Cx - 2, Cx + 2, Cy - 2, Cy + 2, 0);
				ROD::DrawBox(Show3F1MonoImg_planes[2], Cx - 2, Cx + 2, Cy - 2, Cy + 2, 0);

				// Dorysuj okr¹g aproksymuj¹cy kontur jab³ka
				ROD::DrawCircle(Show3F1MonoImg_planes[2], radiusMono, Cx, Cy, DIRS, 250) ;		
				ROD::DrawCircle(Show3F1MonoImg_planes[1], radiusMono, Cx, Cy, DIRS, 100) ;
			    
				cv::merge(Show3F1MonoImg_planes, Show3F1MonoImg);
			    cv::imshow("A5.2 Kontur w RGB/Mono", Show3F1MonoImg);
			    if (LOGS == 1)
			    {
			    	sprintf(outName, "%s/A5_2_KonturRGB_%s.png", ouDir, ouName);
			    	cv::imwrite(outName, Show3F1MonoImg);
			    }
			    
				Show3F1MonoImg.release();
				Show3F1MonoImg_planes[0].release();
				Show3F1MonoImg_planes[1].release();
				Show3F1MonoImg_planes[2].release();
			}
			
			// Wizualizacja funkcji 1D odleg³oœci punktów konturu
			// TO DO
			if (VISUAL == 1) {
				// figId = figId +1;
				// figure(figId); clf;
				// subplot(2,1,1);
				// plot(Kontur1D);
				// title('Kontur 1D');
				// subplot(2,1,2);
				// plot(GradKont1D);
				// title('Gradient konturu 1D'); 
				if (LOGS == 1) {
					// Name = sprintf('log2/Kont1DRgb_%s.png', outName);
					// print ('-dpng', Name);
				}
			}

			

		    //% 5.3 Analiza funkcji 1D konturu w obrazie RGB/Mono (WKas)
		    //% Zauwa¿amy, ¿e detekcja konturu w obrazie IR jest znacznie lepsza ni¿ w
		    //% obrazie RGB. Dlatego szypu³ka w obrazie RGB bêdzie poszukiwana w wycinku
		    //% ko³a zbli¿onym do k¹ta dla szypu³ki znalezionej najpierw w obrazie IR
		   
			cv::Mat roiSzyp= cv::Mat::zeros(6, 1, CV_32SC1);
		    
			if (jestOgonIr0 == 1) // w obrazie IR szypu³ka zosta³a znaleziona
		    {
		    	int w0,w1;
		        DWAPI = 2 * PI;
		        w0 = round(((ROD::alfaIr.at<double>(3,0) + PI) / DWAPI) * DIRS);
		        w1 = round(((ROD::alfaIr.at<double>(2,0) + PI) / DWAPI) * DIRS);
		        if (w0 < 0)    	w0 = DIRS + w0;
		        if (w0 >= DIRS)   w0 = w0 - DIRS;
				if (w1 < 0)     	w1 = DIRS + w1;
		        if (w1 >= DIRS)    w1 = w1 - DIRS;
		        wycinekPoIr = (cv::Mat_<int>(3,1) << w0, w1, DIRS); //% wycinek przy szypu³ce
		    }
		    else
		    	wycinekPoIr = (cv::Mat_<int>(3,1) <<0, DIRS-1, DIRS);//% pe³ny k¹t 2*pi

		    //[jestOgon, pozOgon, alfa, CxNew, CyNew, MaskaOgon, BBox] = ...
			int CxN, CyN;
			::cv::Mat BBox;
			double minDouble, maxDouble;
			// Szukaj w RGB tylko wtedy, gdy by³ ogonek (szyp. zewnêtzna) w IR
			// Uwaga: wspPIK zamieniony ze 120.0
			if (jestOgonIr0 == 1)
				BBox = ROD::OgonekIR(cyf1, cxf1, /*120.0*//*4.8*/20.0, Cx, Cy, wycinekPoIr, Kontur, Kontur1D, Grad1D,
				CxN, CyN);
			else {
				CxN = Cx;
				CyN = Cy;
				BBox = ::cv::Mat::zeros(5, 1, CV_32SC1);
				// ROI obszaru jab³ka nie zmienia siê
				BBox.at<int>(4,0)=  radiusMono;
				cv::minMaxLoc(Kontur.col(0),&minDouble,&maxDouble,0,0,cv::Mat());
				BBox.at<int>(0,0) = round(minDouble); // xmin
				BBox.at<int>(2,0) = round(maxDouble);  // xmax
				cv::minMaxLoc(Kontur.col(1),&minDouble,&maxDouble,0,0,cv::Mat());
				BBox.at<int>(1,0) = round(minDouble); // ymin
				BBox.at<int>(3,0)= round(maxDouble); // ymax
				// Nie ma nowej detekcji ogonka
				jestOgonIr = 0;
			}

			if (MONITOR == 1) { 
				cout<< "Cx, Cy, radiusMono: " << Cx << ", " << Cy << ", " << radiusMono<< endl;
				cout<< "New CxN, CyN: " << CxN << ", " << CyN << endl;
			}

	//pozIrOgon.at<int>(0,0) << "," <<pozIrOgon.at<int>(1,0) <<endl;

		    //% Utwórz ROI szypu³ki - teraz w obrazie w RGB
			// Jednak korzystaliœmy z funkcji OgonekIR - dlatego zmienne nazywaj¹ siê: jestOgonIR, pozIrOgon
		    if (jestOgonIr == 1)
			{
				roiSzyp.at<int>(0,0) =(pozIrOgon.at<int>(0,0) - 20); // xmin
				if (roiSzyp.at<int>(0,0) < 0) 
					roiSzyp.at<int>(0,0) = 0;
				roiSzyp.at<int>(1,0) =(pozIrOgon.at<int>(1,0) - 20); // ymin
				if (roiSzyp.at<int>(1,0) < 0) 
					roiSzyp.at<int>(1,0) = 0;
				roiSzyp.at<int>(2,0) =(pozIrOgon.at<int>(0,0) + 20); // xmax
				if (roiSzyp.at<int>(2,0) >= sx) 
					roiSzyp.at<int>(2,0) = sx-1;
				roiSzyp.at<int>(3,0) =(pozIrOgon.at<int>(1,0) + 20); // ymax
				if (roiSzyp.at<int>(3,0) >= sy) 
					roiSzyp.at<int>(3,0) = sy-1;
				roiSzyp.at<int>(4,0) = pozIrOgon.at<int>(0,0); // xc ogonka
				if (roiSzyp.at<int>(4,0) >= sx) 	roiSzyp.at<int>(4,0) = sx-1;
				roiSzyp.at<int>(5,0) = pozIrOgon.at<int>(1,0); // yc ogonka
				if (roiSzyp.at<int>(5,0) >= sy) 	roiSzyp.at<int>(5,0) = sy-1;
			}
			else if (jestOgonIr0 == 1) // przeniesienie ROI szypu³ki w IR na RGB
			{
				int dx = CirxN - CxN; 
				int dy = CiryN - CyN;

				roiSzyp.at<int>(0,0) = (roiSzypIr.at<int>(0,0) - dx); // wirt xmin
				if (roiSzyp.at<int>(0,0) < 0) roiSzyp.at<int>(0,0) = 0; 
				roiSzyp.at<int>(1,0) = (roiSzypIr.at<int>(1,0) - dy); // wirt ymin
				if (roiSzyp.at<int>(1,0) < 0) roiSzyp.at<int>(1,0) = 0;
				roiSzyp.at<int>(2,0) = (roiSzypIr.at<int>(2,0) - dx); // wirt xmax
				if (roiSzyp.at<int>(2,0) >= sx ) roiSzyp.at<int>(2,0) = sx -1; 
				roiSzyp.at<int>(3,0) = (roiSzypIr.at<int>(3,0) - dy); // wirt ymax
				if (roiSzyp.at<int>(3,0) >= sy) roiSzyp.at<int>(3,0) = sy -1;
				roiSzyp.at<int>(4,0) = (roiSzypIr.at<int>(4,0) - dx); // wirt xc
				if (roiSzyp.at<int>(4,0) >= sx ) roiSzyp.at<int>(4,0) = sx -1; 
				roiSzyp.at<int>(5,0) = (roiSzypIr.at<int>(5,0) - dy); // wirt yc
				if (roiSzyp.at<int>(5,0) >= sy) roiSzyp.at<int>(5,0) = sy -1;
			}
			else { // Zerujemy dla zapewnienia istnienia danych 
				roiSzyp.at<int>(0,0) = 0;
				roiSzyp.at<int>(1,0) = 0;
				roiSzyp.at<int>(2,0) = 0;
				roiSzyp.at<int>(3,0) = 0;
				roiSzyp.at<int>(4,0) = 0;
				roiSzyp.at<int>(5,0) = 0;
			}

		    // Wizualizacja konturu w obrazie
		    std::vector<cv::Mat> Show3F1MonoImg_planes(3);
		    cv::Mat Show3F1MonoImg;   
		    
			// Wizualizacja konturu w obrazie Mono
		    if (VISUAL == 1)
		    {
		    	int x,y;
		    	//cv::split(F1ColorImg,Show3F1MonoImg_planes);
		    	Show3F1MonoImg_planes[0] = F1MonoImg.clone();//uchar
		    	Show3F1MonoImg_planes[1] = F1MonoImg.clone();
		    	Show3F1MonoImg_planes[2] = F1MonoImg.clone();

		    	for (int i=0;i<DIRS;i++)
		    	{
		    		y = int(Kontur.at<double>(i,1) );
		    		x = int(Kontur.at<double>(i,0) );

		    		if (MaskaIrOgon.at<uchar>(i,0) == 1)
		    		{
		    			// fragment konturu to szypu³ka
		    			Show3F1MonoImg_planes[0].at<uchar>(y,x) = 255;
		    			Show3F1MonoImg_planes[1].at<uchar>(y,x) = 255;
		    			Show3F1MonoImg_planes[2].at<uchar>(y,x) = 255;
		    		}
		    		else
		    		{
		    			// rzeczywisty kontur obszaru jab³ka
		    			Show3F1MonoImg_planes[0].at<uchar>(y,x) = 255;
		    			Show3F1MonoImg_planes[1].at<uchar>(y,x) = 100;
		    			Show3F1MonoImg_planes[2].at<uchar>(y,x) = 0;
		    		}
		    	}
					// Ewentualnie dorysuj ROI ogonka
				if ( (jestOgonIr == 1) || (jestOgonIr0 == 1) )
				{
					ROD::DrawBox(Show3F1MonoImg_planes[0], roiSzyp.at<int>(0,0), roiSzyp.at<int>(2,0), roiSzyp.at<int>(1,0), roiSzyp.at<int>(3,0),100);
					ROD::DrawBox(Show3F1MonoImg_planes[1], roiSzyp.at<int>(0,0), roiSzyp.at<int>(2,0), roiSzyp.at<int>(1,0), roiSzyp.at<int>(3,0),100);
					ROD::DrawBox(Show3F1MonoImg_planes[2], roiSzyp.at<int>(0,0), roiSzyp.at<int>(2,0), roiSzyp.at<int>(1,0), roiSzyp.at<int>(3,0),255);

				// Dorysuj osie - wycinek poszukiwania
					DrawLine(wycinekPoIr.at<int>(0,0) * DWAPI/DIRS , 255, CxN, CyN, Show3F1MonoImg_planes[0]); // b plane
					DrawLine(wycinekPoIr.at<int>(1,0) * DWAPI/DIRS , 255, CxN, CyN, Show3F1MonoImg_planes[0]); // b plane
					DrawLine(wycinekPoIr.at<int>(0,0) * DWAPI/DIRS , 150, CxN, CyN, Show3F1MonoImg_planes[1]); // g plane
					DrawLine(wycinekPoIr.at<int>(1,0) * DWAPI/DIRS , 150, CxN, CyN, Show3F1MonoImg_planes[1]); // g plane
					DrawLine(wycinekPoIr.at<int>(0,0) * DWAPI/DIRS , 50, CxN, CyN, Show3F1MonoImg_planes[2]); // r plane
					DrawLine(wycinekPoIr.at<int>(1,0) * DWAPI/DIRS , 50, CxN, CyN, Show3F1MonoImg_planes[2]); // r plane
				}

				// Dorysuj œrodek konturu
				ROD::DrawBox(Show3F1MonoImg_planes[0], CxN - 2, CxN + 2, CyN - 2, CyN + 2, 255);
				ROD::DrawBox(Show3F1MonoImg_planes[1], CxN - 2, CxN + 2, CyN - 2, CyN + 2, 0);
				ROD::DrawBox(Show3F1MonoImg_planes[2], CxN - 2, CxN + 2, CyN - 2, CyN + 2, 0);

				// Dorysuj okr¹g aproksymuj¹cy kontur jab³ka
				ROD::DrawCircle(Show3F1MonoImg_planes[2], radiusMono, CxN, CyN, DIRS, 250) ;		
				ROD::DrawCircle(Show3F1MonoImg_planes[1], radiusMono, CxN, CyN, DIRS, 100) ;
				//

		    	cv::merge(Show3F1MonoImg_planes,Show3F1MonoImg);
		    	
				cv::imshow("A5.3 Kontur z szypu³k¹ w RGB/Mono", Show3F1MonoImg);
		    	if (LOGS == 1)
		    	{
		    		sprintf(outName, "%s/A5_3_KontRGBSzyp_%s.png", ouDir, ouName);
		    		cv::imwrite(outName, Show3F1MonoImg);
		    	}
		    	//Show3F1MonoImg.release();
				//Show3F1MonoImg_planes[0].release();
				//Show3F1MonoImg_planes[1].release();
				//Show3F1MonoImg_planes[2].release();
		    }

			//cv::waitKey();

		    //%%
		    //% 5.4 Detekcja szypu³ki wewnêtrznej lub granicznej
		    //% - na podstawie obrazu krawêdziowego i obrazu Mono
		    //%  Najlepiej przy wczeœniejszej detekcji szypu³ki odstaj¹cej

		    if (jestOgonIr == 1) // wykryty w obrazie RGB 
		    {
		    	int w0,w1;
		        DWAPI = 2 * PI;
		        w0 = round(((alfaIr.at<double>(3,0) + PI) / DWAPI) * DIRS);
		        w1 = round(((alfaIr.at<double>(2,0) + PI) / DWAPI) * DIRS);
		        
				if (w0 < 0)
		        	w0 = DIRS + w0;
				if (w0 >=DIRS) w0 = w0 - DIRS;
				if (w1 < 0)
					w1 = DIRS + w1;
		        if (w1 >= DIRS)
		            w1 = w1 - DIRS;
		        wycinek = (cv::Mat_<int>(3,1)<<w0, w1, DIRS);// % wycinek przy szypu³ce
		    }
		    else if (jestOgonIr0 == 1) // wykryty w obrazie IR
			{
				wycinek = wycinekPoIr.clone();// % wycinek przy szypu³ce
			}
			else
		    	wycinek = (cv::Mat_<int>(3,1)<< 0, DIRS-1, DIRS);//% pe³ny k¹t 2*pi


			// niejawny wynik MaskaMonoSzyp; // dla wizualizacji wyniku detekcji obszaru szypu³ki
		    
			//if (jestOgonIr == 1)
			// w obrazie mono
		    	ROD::SzypulkaMono(CxN, CyN, Kontur, Kontur1D, F1MonoImg, F1EdgeMImg, F1EdgeDirImg, wycinek, radiusMono, 
					0.20, 100);
			// w obrazie IR
			//      ROD::SzypulkaMono(CirxN, CiryN, IrKontur, IrKontur1D, NormIRImg, EdgeIRMImg, EdgeIRDirImg,
			//		wycinek, 0.20, 100);
		    //else {
		     //   jestSzypROD = 0; 
			//	roiSzypMono = (cv::Mat_<int>(4,1) << 0,0,0,0);
		     //   roiSzypMono.release();//roiSzypMono = [];
		    //}

			// TEST
			//cout << jestSzypROD << endl;
			// 

		    if (VISUAL == 1)
		    {
		        if (jestSzypROD == 1)
		        {
					// Wizualizacja krawêdzi zgodnych z szypu³k¹
		        	cv::imshow("A5.4 Krawêdzie zgodne z szypu³k¹ wewn", MaskaMonoSzyp);

		        	if (LOGS == 1)
		        	{
						sprintf(outName, "%s/A5_4_SzypMonoKraw_%s.png", ouDir, ouName);
		        		cv::imwrite(outName, MaskaMonoSzyp);
		        	} 
					
					// Dorysuj ROI szypu³ki wewnêtrznej
					ROD::DrawBox(Show3F1MonoImg_planes[0], roiSzypMono.at<int>(0,0), roiSzypMono.at<int>(2,0),roiSzypMono.at<int>(1,0),roiSzypMono.at<int>(3,0),225);
					ROD::DrawBox(Show3F1MonoImg_planes[1], roiSzypMono.at<int>(0,0), roiSzypMono.at<int>(2,0),roiSzypMono.at<int>(1,0),roiSzypMono.at<int>(3,0),155);
					
					cv::merge(Show3F1MonoImg_planes, Show3F1MonoImg /*MonoE3Img*/);
					cv::imshow("A5.4a Kontur mono i szyp wewn", Show3F1MonoImg /*MonoE3Img*/);
					if (LOGS == 1)
					{
						sprintf(outName, "%s/A5_4a_Kont_SzypWewn_%s.png", ouDir, ouName);
						cv::imwrite(outName, Show3F1MonoImg /*MonoE3Img*/);
					}
					 
		        }
		        else if ((jestOgonIr == 1) || (jestOgonIr0 ==1) )
		        {
		        	roiSzypMono = roiSzyp.clone(); // jeœli by³ ogonek odstaj¹cy (znaleziony funkcj¹ w ROD)
		        }
		    }

			Show3F1MonoImg.release();
			Show3F1MonoImg_planes[0].release();
			Show3F1MonoImg_planes[1].release();
			Show3F1MonoImg_planes[2].release();
			
			if (jestSzypROD == 1) MaskaMonoSzyp.release();
		    
			// Koniec pomiaru czasu dla kroku 5
		    end=time(0);
		    elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
		    cout << "Czas kroku 5: " << endl;
		    cout << elapsed<< " ms" <<  endl;
		    cout << end-start << " s" <<  endl;


			//////////////////////
		    // 6. Analiza 2D obrazu IR (AW)
		    //
			int jestOgon2, jestSzypWewn;
			FeatureDescriptor szypulkaOdst, szypulkaWewn;

		    t0=clock();             // zliczanie czasu
		    start=time(0);

			if (STEMsw == 1)  {
				szypulkaOdst = OdstajacaSzypulka(ouName, VISUAL, LOGS, ColorImg); // (A. Wilkowski)
				jestOgon2 = szypulkaOdst.decision;
			}
			else {
				jestOgon2 = 0;
				szypulkaOdst.decision = 0;
			}

		    
			// Koniec pomiaru czasu dla kroku 6
		    end=time(0);
		    elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
		    cout << "Czas kroku 6: " << endl;
		    cout << elapsed<< " ms" <<  endl;
		    cout << end-start << " s" <<  endl;


			///////////////////
		    //% 7. Detekcja zag³êbieñ w Mono (AW)
		    t0=clock();             // zliczanie czasu
		    start=time(0);
			
			if (STEMsw == 1) {
				szypulkaWewn = SzypulkaWObrysie(ouDir, ouName, VISUAL, LOGS, ColorImg, param.relDetThresh, 
								param.backgroundThresh, param.selStemPairRatio );//
				jestSzypWewn = szypulkaWewn.decision;
			}
			else
			{
				jestSzypWewn = 0; 
				szypulkaWewn.decision = 0;
			}

			// Koniec kroku 7
		   
			

			//////////////////////////
			// 7.A Podsumuj wyniki detekcji ogonka i szypu³ki w IR i RGB 
			// wykonane poprzez analizê 1D (ROD) i analizê 2D (STEM)

			ROD::ResultOD result = ROD::ResultOD();

			// Zapamiêtujemy wyniki w strukturze zwracanej przez funkcjê
			// Œrodek konturu RGB
			result.cx = CirxN; // Uwaga: w uk³adzie obrazu !
			result.cy = CiryN; // --" --
			// Promieñ okrêgu aproksymuj¹cego kontur
			result.radius = radiusMono;
			
			if ((jestOgonIr==1) || (jestOgonIr0 == 1) || (jestOgon2 == 1))
			{
				result.jestOgon = 1; // odstaj¹cy ogonek w obrazie Mono (metodami w ROD) lub metod¹ w STEM 
				result.ogonCx = roiSzyp.at<int>(4,0); // po³o¿enie nasady szypu³ki
				result.ogonCy = roiSzyp.at<int>(5,0);
			}
			else
			{
				result.jestOgon = 0;
				result.ogonCx = 0; // po³o¿enie nasady szypu³ki
				result.ogonCy = 0;
			}

			if ((jestSzypROD == 1) || (jestSzypWewn == 1))
				result.jestSzyp = 1; // szypu³ka wewnêtrzna
			else 
				result.jestSzyp = 0;

			result.bBox[0] = BBox.at<int>(0,0);// IRcolumnMin; // minX - prostok¹t obejmuj¹cy jab³ko
			result.bBox[2] = BBox.at<int>(2,0); // IRcolumnMax; // maxX
			result.bBox[1] = BBox.at<int>(1,0); // minY
			result.bBox[3] = BBox.at<int>(3,0); // maxY

			// Koniec pomiaru czasu dla kroku 7
		    end=time(0);
		    elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
		    cout << "Czas kroku 7: " << endl;
		    cout << elapsed<< " ms" <<  endl;
		    cout << end-start << " s" <<  endl;
			
			//

			if (VISUAL == 1) cv::destroyAllWindows();

		    //////////////////////////////////////////////////////////////
		    //
		    // III. ANALIZA kolorowa OBSZARU WNÊTRZA obrazu RGB --> YCgCr
		    // 
		    ///////
			
			// 8. Maska wnêtrza obrazu
		    t0=clock();             // zliczanie czasu
		    start=time(0);
		    //
		    // 8.1.a Maska wnêtrza obrazu RGB i maskowanie obrazu RGB
			cv::Mat  MaskImg = ROD::MaskaObrazuRGB(Cy, Cx, F1ColorImg, F1MonoImg, Kontur);//, 80, 60);
		    // Wizualizacja maskowanego obrazu
		    if (VISUAL == 1)
		    {
		    	cv::imshow("8.1a Maska w obrazie mono",F1MonoImg);
		    	cv::imshow("8.1b Maska w obrazie RGB",F1ColorImg);
		    }

		    if (LOGS == 1)
		    {
		    	sprintf(outName, "%s/A8_1_F1Mono_%s.png", ouDir, ouName);
		    	cv::imwrite(outName, F1MonoImg);
		    	sprintf(outName, "%s/A8_1_F1Color_%s.png", ouDir, ouName);
		    	cv::imwrite(outName, F1ColorImg);
		    }

		    //
		    // 8.1.b Kszta³t w obrazie IR
		    // Œrodek konturu w nowym obrazie:
		    int CiryNew = CiryN;
		    int CirxNew = CirxN;
			//int F1Ciry = CiryNew - (int)IRrowMin;// + 1;
			//int F1Cirx = CirxNew - (int)IRcolumnMin; // + 1;
			int F1Ciry = CiryNew ;
			int F1Cirx = CirxNew ;
		    
			// Kontur w nowym obrazie
		    //for (int i=0;i< DIRS;i++)
		    //{
		    //    IrKontur.at<double>(i,0) = IrKontur.at<double>(i,0) - IRcolumnMin;
		    //    IrKontur.at<double>(i,1) = IrKontur.at<double>(i,1) - IRrowMin;
		    //}

		    // Maskuj obraz IR
			//cv::Mat NormIRImgROI = NormIRImg(cv::Rect((int)IRcolumnMin, (int)IRrowMin, (int)(IRcolumnMax - IRcolumnMin),
			//	(int)(IRrowMax - IRrowMin))).clone();
			cv::Mat F1IRImg = NormIRImg.clone();
			cv::Mat IRMaskImg = ROD::MaskaObrazuIR(F1Ciry, F1Cirx, F1IRImg, IrKontur); //, 80);

		    // Wizualizacja maski obrazu IR
		     if (VISUAL == 1)
			{
		    	cv::imshow("A8.1c Maska w obrazie IR", F1IRImg);
			}
			// Utrwalenie obrazu wynikowego
		    if (LOGS == 1)
		    {
		    	sprintf(outName, "%s/A8_1_F1IR_%s.png", ouDir, ouName);
		    	cv::imwrite(outName, /*F1MonoImg*/F1IRImg);
		    }

		    //%%%%%%%%%%%%%%%
		    // Krok 8.2: Oblicz histogram obszaru w obrazie RGB
		    //%%%%%%%%%%%%%%%
		    // Histogram RGB (piksele obszaru jedynie - maska= 0)
		    int mcy, mcx;
		    mcy = MaskImg.rows;
		    mcx = MaskImg.cols;

		    // Wyznaczanie i wizualizacja 3 histogramów
		    if (VISUAL == 1)
		    {
		    	cv::Mat histoRed,histoBlue,histoGreen;
		    	std::vector<cv::Mat> F1ColorImg_planes(3);
		    	cv::split(F1ColorImg,F1ColorImg_planes);
		    	//cv::imshow("TEST",F1ColorImg_planes[0]);
		    	//RED
		    	::cv::calcHist(&F1ColorImg_planes[2], 1, 0, 255-255*MaskImg, histoRed, 1, &histSize, 0);
		    	::cv::Mat histImageR = ::cv::Mat::ones(256, 256, CV_8U)*255;
		    	::cv::normalize(histoRed, histoRed, 0, histImageR.rows, CV_MINMAX, CV_32F);
		    	int binW = cvRound((double)histImage.cols/histSize);
		    	for( int i = 0; i < histSize; i++ )
		    		rectangle( histImageR, ::cv::Point(i*binW, histImageR.rows),
		    				::cv::Point((i+1)*binW, histImageR.rows - cvRound(histoRed.at<float>(i))),
		    				 ::cv::Scalar::all(0), -1, 8, 0 );
		    	::cv::imshow("A8.2a Histogram Red", histImageR);
		    	histImageR.release();

		    	//Green
		    	::cv::calcHist(&F1ColorImg_planes[1], 1, 0, 255-255*MaskImg, histoGreen, 1, &histSize, 0);
		    	::cv::Mat histImageG = ::cv::Mat::ones(256, 256, CV_8U)*255;
		    	::cv::normalize(histoGreen, histoGreen, 0, histImageG.rows, CV_MINMAX, CV_32F);
		    	for( int i = 0; i < histSize; i++ )
		    		rectangle( histImageG, ::cv::Point(i*binW, histImageG.rows),
		    				::cv::Point((i+1)*binW, histImageG.rows - cvRound(histoGreen.at<float>(i))),
		    				 ::cv::Scalar::all(0), -1, 8, 0 );
		    	::cv::imshow("A8.2b Histogram Green", histImageG);
		    	histImageG.release();

		    	//Blue
		    	::cv::calcHist(&F1ColorImg_planes[0], 1, 0, 255-255*MaskImg, histoBlue, 1, &histSize, 0);
		    	::cv::Mat histImageB = ::cv::Mat::ones(256, 256, CV_8U)*255;
		    	::cv::normalize(histoBlue, histoBlue, 0, histImageB.rows, CV_MINMAX, CV_32F);
		    	for( int i = 0; i < histSize; i++ )
		    		rectangle( histImageB, ::cv::Point(i*binW, histImageB.rows),
		    				::cv::Point((i+1)*binW, histImageB.rows - cvRound(histoBlue.at<float>(i))),
		    				 ::cv::Scalar::all(0), -1, 8, 0 );
		    	::cv::imshow("A8.2c Histogram Blue", histImageB);
		    	histImageB.release();
		    	//histoRed.release();histoBlue.release();histoGreen.release();
		    }

		    //
		    //%%%%%%%%%%%%%%%%%%%%%%
		    // 8.3: Zamieñ RGB na Y CG Cr (uwaga: alternatywnie YCbCr ?)
		    // i normalizuj wybrany kolor wzglêdem jasnoœci
		    //%%%%%%%%%%%%%%%%%%%%%%
			int maxYValue; // maksymalna jasnoœæ w obrazie po przekszta³ceniu
		    
			// 8.3.a RGB --> YCbCr (YCgCr)
		    //mcy = MaskImg.rows;
		    //mcx = MaskImg.cols;

		    ROD::YbrImg = ROD::RGBtoYbr(mcy, mcx, 0, &maxYValue, F1ColorImg, MaskImg).clone(); 
			// Parametr : 0= YCgCr, 1 = YCbCr

		    std::vector<cv::Mat> YbrImg_split;
		    cv::split(YbrImg, YbrImg_split);
			
			// Wizualizacja i utrwalenie obrazu YCbCr i jego sk³adowych
		    if (VISUAL == 1)
		    {
		    	cv::imshow("A8.3a Obraz YCbCr (YCgCr)",YbrImg);
		    	//Y
		    	cv::imshow("A8.3b Sk³adowa Y",YbrImg_split[0]);
		    	//CB (CG) - U component
		    	cv::imshow("A8.3c Sk³adowa CB (CG)",YbrImg_split[1]);
		    	//CR - V component
		    	cv::imshow("A8.3d Sk³adowa CR",YbrImg_split[2]);

		    }
		    if (LOGS == 1)
		    {
		    	sprintf(outName, "%s/A8_3a_YUV_%s.png", ouDir, ouName);
		    	cv::imwrite(outName, YbrImg);
		    	sprintf(outName, "%s/A8_3b_Y_%s.png", ouDir, ouName);
		    	cv::imwrite(outName, YbrImg_split[0]);
		    	sprintf(outName, "%s/A8_3c_U_%s.png", ouDir, ouName);
		    	cv::imwrite(outName, YbrImg_split[1]);
		    	sprintf(outName, "%s/A8_3d_V_%s.png", ouDir, ouName);
		    	cv::imwrite(outName, YbrImg_split[2]);
		    }


		    // 8.3.b Oblicz histogram obrazu Y Cb Cr
		    //cv::Mat HYbrImg_1;
		    cv::Mat HYbrImg_2;
		    cv::Mat HYbrImg_3;
		    double maxCb, maxCr;
		    cv::Point indMaxCb, indMaxCr;

			//::cv::calcHist(&YbrImg_split[0], 1, 0, 255 - 255 * MaskImg, HYbrImg_1, 1, &histSize, 0);
			::cv::calcHist(&YbrImg_split[1], 1, 0, 255 - 255 * MaskImg, HYbrImg_2, 1, &histSize, 0);
			::cv::calcHist(&YbrImg_split[2], 1, 0, 255 - 255 * MaskImg, HYbrImg_3, 1, &histSize, 0);


		    // 8.3.c Automatycznie ustal typ jab³ka ze wzglêdu na kolor dominuj¹cy
		    HYbrImg_2.pop_back(1); HYbrImg_3.pop_back(1);
		    ::cv::minMaxLoc(HYbrImg_2,0, &maxCb, 0, &indMaxCb,cv::noArray());
		    ::cv::minMaxLoc(HYbrImg_3,0, &maxCr, 0, &indMaxCr,cv::noArray());
		    int aType;
		    if ((indMaxCb.y + 40) < indMaxCr.y)
		        aType =  1 ; // czerwone jab³ko
		    else
		    {
		        if ((indMaxCb.y + 10) < indMaxCr.y)
		            aType = 3; // ¿ó³te jab³ko
		        else
		            aType = 2; // zielone jab³ko
		    }
			// TEST
			//cout<< "indMaxCb, indMaxCr, aType:" << indMaxCb << ", " << indMaxCr << ", " << aType << endl;
			//cout<< "indMaxCb.y, indMaxCr.y, aType:" << indMaxCb.y << ", " << indMaxCr.y << ", " << aType << endl;//
			//cv::waitKey();

		    //HYbrImg_1.release();
		    HYbrImg_2.release();
		    HYbrImg_3.release();


		    // 8.4 W zaleznoœci od typu wykonaj normalizacjê wzglêdem jasnoœci
		    if (aType ==1)
		    {
		        // normY = 66
		    	YbrImg = ROD::NormYBR(mcy, mcx, -0.3333, 0.60, maxYValue, 66.0, YbrImg, MaskImg).clone();
		    // Zamieñ i normalizuj wzglêdem jasnoœci (z pominiêciem pikseli zewn. =1 i brzegu =2)
		    // Dla CG CR i czerwonego koloru jab³ka zak³adamy wspó³czynniki: KB = -0.3333, KR = 0.60,
		    // DLa CB, CR: KB = 0.1, KR = 0.3
		    // przeskalowania sk³adowych wzglêdem zmiany jasnoœci.
		    }
		    else
		    {
		        if (aType == 2)
		        //% Dla zielonego jab³ka : KB = 0.02, KR = 0.10; normY = 120
		        	YbrImg = ROD::NormYBR(mcy, mcx, -0.17, 0.30, maxYValue, 120.0, YbrImg, MaskImg).clone();
		        else
		        {
		            //% Dla ¿ó³tego jab³ka : normY = 120
		        	YbrImg = ROD::NormYBR(mcy, mcx, -0.17, 0.30, maxYValue, 120.0, YbrImg, MaskImg).clone();
		        }
		    }

		    // wizualizacja obrazu YCbCr i jego sk³adowych
		    cv::split(YbrImg, YbrImg_split);
		    if (VISUAL == 1)
		    {
		    	cv::imshow("8.4a Obraz YCbCr (YCgCr)",YbrImg);
		    	//Y
		    	cv::imshow("8.4b Sk³adowa Y",YbrImg_split[0]);
		    	//CB - U component
		    	cv::imshow("8.4c Sk³adowa CB (CG) norm",YbrImg_split[1]);
		    	//CR - V component
		    	cv::imshow("8.4d Sk³adowa CR norm",YbrImg_split[2]);
		    }
		    if (LOGS == 1)
		    {
		    	sprintf(outName, "%s/A8_4a_YUV_norm_%s.png", ouDir, ouName);
		    	cv::imwrite(outName, YbrImg);
		    	sprintf(outName, "%s/A8_4b_Ynorm_%s.png", ouDir, ouName);
		    	cv::imwrite(outName, YbrImg_split[0]);
		    	sprintf(outName, "%s/A8_4c_Unorm_%s.png", ouDir, ouName);
		    	cv::imwrite(outName, YbrImg_split[1]);
		    	sprintf(outName, "%s/A8_4d_Vnorm_%s.png", ouDir, ouName);
		    	cv::imwrite(outName, YbrImg_split[2]);
		    }

		    //
		    // 8.4b Oblicz ponownie histogram obrazu Y Cb Cr po normalizacji
		    //HYbrImg = ROD.MaskedHisto(mcy, mcx, YbrImg, MaskImg); % z pominiêciem pikseli "zewnêtrznych"
			//::cv::calcHist(&YbrImg_split[0], 1, 0, 255 - 255 * MaskImg, HYbrImg_1, 1, &histSize, 0);
			::cv::calcHist(&YbrImg_split[1], 1, 0, 255 - 255 * MaskImg, HYbrImg_2, 1, &histSize, 0);
			::cv::calcHist(&YbrImg_split[2], 1, 0, 255 - 255 * MaskImg, HYbrImg_3, 1, &histSize, 0);
			//HYbrImg_1.pop_back(1);
			//HYbrImg_2.pop_back(1);
			//HYbrImg_3.pop_back(1);

		    //
		    // 8.5: wariancja sk³adowych Cb, Cr w blokach obrazu
		    //
			cv::Mat StdHist;
		    StdHist = ROD::MaskedVarianceHisto(mcy, mcx, YbrImg, MaskImg).clone();
			// zwraca niejawnie StdImgG i StdImgR;

		    // Kontrolna wizualizacja histogramów wariancji bloków
		    if (VISUAL == 1)
		    {
		    	cv::Mat histImage = ::cv::Mat::ones(256, 256, CV_8U)*255;
		    	int binW = cvRound((double)histImage.cols/histSize);
		    	for( int i = 0; i < histSize; i++ )
		    		cv::rectangle( histImage, ::cv::Point(i*binW, histImage.rows),
		    				::cv::Point((i+1)*binW, histImage.rows - cvRound(StdHist(cv::Rect(0,0,1,StdHist.rows)).at<float>(i))),
		    				 ::cv::Scalar::all(0), -1, 8, 0 );
		    	::cv::imshow("8.5a Histogram wariancji CG (U)", histImage);
		    	histImage = ::cv::Mat::ones(256, 256, CV_8U)*255;
		    	for( int i = 0; i < histSize; i++ )
		    		cv::rectangle( histImage, ::cv::Point(i*binW, histImage.rows),
		    				::cv::Point((i+1)*binW, histImage.rows - cvRound(StdHist(cv::Rect(1,0,1,StdHist.rows)).at<float>(i))),
		    				 ::cv::Scalar::all(0), -1, 8, 0 );
		    	::cv::imshow("8.5b Histogram wariancji Cr (V)", histImage);
		    	histImage.release();

		    }

		    //% Wizualizacja rozk³adu wariancji w obrazie
		    if (VISUAL == 1)
		    {
		    	cv::imshow("8.5c Wariancja sk³adowej CG (U)",StdImgG);
		    	cv::imshow("8.5d Wariancja sk³adowej CR (V)",StdImgR);

		        if (LOGS == 1)
		        {
		        	sprintf(outName, "%s/A8_5c_StdU_%s.png", ouDir, ouName);
		        	cv::imwrite(outName, StdImgG);
		        	sprintf(outName, "%s/A8_5d_StdV_%s.png", ouDir, ouName);
		        	cv::imwrite(outName, StdImgR);
		        }
		    }

		    // 8.5b Po³o¿enie maksimów w histogramach
			cv::Point PointMax;
		    int maxGval, maxRval; // wartoœci o najwiêkszej czêstoœci w obrazie
		    double Y;
			
			cv::minMaxLoc(HYbrImg_2, 0, &Y, 0, &PointMax, cv::Mat());
			maxGval = round(PointMax.y);//  + 1 + (int)PointMax.x*HYbrImg_2.cols);
			// TEST
			//cout<< "2: PointMax, maxGval:"<< PointMax << ", "<< maxGval << endl;
			
			cv::minMaxLoc(HYbrImg_3, 0, &Y, 0, &PointMax, cv::Mat());
			maxRval = round(PointMax.y); //  + 1 + (int)PointMax.x*HYbrImg_3.cols);
			
			// cout<< "3: PointMax, maxRval:"<< PointMax << ", " << maxRval << endl;
			
			//cv::waitKey();

		    ///
		    // 8.6: Detekcja obszarów jednorodnych wewn¹trz konturu jab³ka
		    SegmentImg = ROD::Regions(mcy, mcx, YbrImg_split[1], YbrImg_split[2], MaskImg, StdImgR).clone();
			
			SegmentImg.convertTo(SegmentImg, CV_8UC1);
		    if (VISUAL == 1)
		    {
		        cv::Mat Mono3Img = YbrImg.clone();
		        std::vector<cv::Mat> Mono3Img_planes;
		        cv::split(Mono3Img,Mono3Img_planes);
		        Mono3Img_planes[0] = MaskImg * 127;
				Mono3Img_planes[2] = SegmentImg/4.0 + MaskImg * 127;//round(SegmentImg /4.0);
				Mono3Img_planes[1] = SegmentImg + MaskImg * 127;//round(SegmentImg);
		        cv::merge(Mono3Img_planes,Mono3Img);
		        cv::imshow("8.6 Obraz obszarów jednorodnych",Mono3Img);

		        if (LOGS == 1)
		        {
		        	sprintf(outName, "%s/A8_6_Region_%s.png", ouDir, ouName);
		        	cv::imwrite(outName, Mono3Img);
		        }
		        Mono3Img.release();
		    }

		    // Koniec pomiaru czasu dla kroku 8
		    end=time(0);
		    elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
		    cout << "Czas kroku 8: " << endl;
		    cout << elapsed<< " ms" <<  endl;
		    cout << end-start << " s" <<  endl;



			////////////// 
 			// Kroki 9-12: analiza 3D obrazu IR.
			// Obszar: SFS
			// Autor: Karolina Przerwa

			std::vector<StatsRoi> rois;

			t0=clock();		// zliczanie czasu
			start=time(0);

			if (SFSsw == 1)
				rois = sfs3D->solution(fIrName, ouDir, ouName); // analiza obrazu IR metodami obiektu Calibrator
				//rois = sfs3D->solution( fIrName, IRImg, ouDir, ouName );

			if (MONITOR == 1) std::cout<< "SFS koniec\n"; 
			
			// Koniec pomiaru czasu dla kroków 9-12
			end=time(0);
			elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
			cout << "Czas kroków (SFS) 9-12: " << endl;
			cout << elapsed<< " ms" <<  endl;
			cout << end-start << " s" <<  endl;
			
			//
			//
			// Krok 13: Klasyfikacja kolorowa 2D i opis wyników
			//
			t0 = clock();		// zliczanie czasu
			start = time(0);

			cv::Mat MeanCRImg;// = cv::Mat::zeros(mcy, mcx, CV_8UC1);
			cv::Mat MeanCGImg;// = cv::Mat::zeros(mcy, mcx, CV_8UC1);

			//filtr dolnoprzepustowy
			//MeanCRImg = ROD::MeanOperator(mcy, mcx, YbrImg_split[2], MaskImg); //%maskowane uœrednianie
			//MeanCGImg = ROD::MeanOperator(mcy, mcx, YbrImg_split[1], MaskImg); //%maskowane uœrednianie
			//mask
			cv::Mat kernelMat = cv::Mat::ones(5, 5, CV_32FC1) / 25;
			cv::filter2D(YbrImg_split[2], MeanCRImg, -1, kernelMat);
			cv::filter2D(YbrImg_split[1], MeanCGImg, -1, kernelMat);
			//for (int j = 0; j < mcy;j++)
			//	for (int i = 0; i < mcx;i++)
			//		if (MaskImg.at<uchar>(j, i) != 0)
			//		{
			//			MeanCRImg.at<uchar>(j, i) = 0;
			//			MeanCGImg.at<uchar>(j, i) = 0;
			//		}

			//
			// 13.1 Na podstawie obrazu kolorowego
			// Klasyfikacja w oparciu o obszary w obrazie RGB
			// [wynikRGB ClassReg ClassImg] = ROD.ClassImage1(mcy, mcx, maxGval, maxRval, F1MonoImg, MaskImg, HYbrImg(:, 2), HYbrImg(:, 3), SegmentImg, regNum, regionDesc, roiSzypMono, aType, uparameter);
			ROD::ClassImage1(mcy, mcx, maxGval, maxRval, F1MonoImg, MaskImg, HYbrImg_2, HYbrImg_3, 
				SegmentImg, regNum, regionDesc, roiSzypMono, aType, uparam);
			// wynik niejawnie zwracany w obiekcie "wynikRGB"

			// Ewentualnie, jeœli by³a przeprowadzona redukcja obrazu RGB do obszaru ROI,
			// powrót do pocz¹tkowego rozmiaru obrazu RGB i po³o¿enia w nim jab³ka
			//std::vector<cv::Mat>ClassCRImg_planes(3), ClassImg_planes(3);
			//cv::split(ClassImg, ClassImg_planes);
			//cv::Mat ClassCRImg;
			//ClassCRImg_planes[0] = 255 * (cv::Mat::ones(ColorImg.rows, ColorImg.cols, CV_8UC1));
			//ClassCRImg_planes[1] = 255 * (cv::Mat::ones(ColorImg.rows, ColorImg.cols, CV_8UC1));
			//ClassCRImg_planes[2] = 255 * (cv::Mat::ones(ColorImg.rows, ColorImg.cols, CV_8UC1));
			//for (int i = 0; i < ClassImg.rows; i++) 
			//	for (int j = 0; j < ClassImg.cols; j++)
			//		for (int k = 0; k < 3; k++)
			//			ClassCRImg_planes[k].at<uchar>(i + rowMin, j + columnMin) = ClassImg_planes[k].at<uchar>(i, j);
			//cv::merge(ClassCRImg_planes, ClassCRImg);
			
			wynikRGB.classImage = ClassImg; // wizualizacja klasyfikacji

			//Ewentualnie przesuñ wspó³rzêdne w obiekcie wyniku do uk³adu oryginalnego obrazu
			//std::cout << "srodek cx class" << crxClass << "\n";
			wynikRGB.cx = crxClass; // /*Cirx*/ + columnMin;
			wynikRGB.cy = cryClass; // /*Ciry*/ + rowMin;
			wynikRGB.bBox[0] = BBox.at<int>(0,0);// + columnMin;
			wynikRGB.bBox[0] = BBox.at<int>(1,0);// + rowMin;
			wynikRGB.bBox[0] = BBox.at<int>(2,0);// + columnMin;
			wynikRGB.bBox[0] = BBox.at<int>(3,0);// + rowMin;

			//for (int i = 0; i < regNum; i++)
			//{
				//regionDesc.at<double>(i, 3) += rowMin;//przesuniecie srodka - œrodek cy
				//regionDesc.at<double>(i, 4) += columnMin;//przesuniecie srodka - œrodek cx
			wynikRGB.regions = regionDesc;
			//}
			
			// Wizualizacja i utrwalenie wyników
			if (VISUAL == 1)
			{
				imshow("13.1 Klasy w RGB (1-5):  ok (ziel), odcisk (szary), plamka (¿ó³ta), grzyb (czerw), szyp (nieb)", ClassImg);
				if (LOGS == 1)
				{
					sprintf(outName, "%s/A13_1_ClassRGB_%s.png", ouDir, ouName);
					cv::imwrite(outName, ClassImg);
				}
			}

			///
			// 13.2.Na podstawie obrazu IR - analiza 2D
			//
			int msy_ = F1IRImg.rows; 
			int msx_ = F1IRImg.cols;
			// Uwaga: rozmiary obrazów RGB i IR musz¹ byc identyczne!
			//cv::Mat ROI = EdgeIRMImg(cv::Rect((int)IRcolumnMin, (int)IRrowMin, (int)(IRcolumnMax - IRcolumnMin), (int)(IRrowMax - IRrowMin))).clone();
			//cv::Mat ROI = EdgeIRMImg.clone();

			ClassIRImage(msy_, msx_, F1Cirx, F1Ciry, roiSzypIr, F1IRImg, IRMaskImg, EdgeIRMImg, ClassImg);

			wynikIr.cx = F1Cirx;// + IRcolumnMin;
			wynikIr.cy = F1Ciry;// + IRrowMin;
			wynikIr.bBox[0] = BBoxIr.at<int>(0, 0);// + IRcolumnMin;
			wynikIr.bBox[1] = BBoxIr.at<int>(1, 0);// + IRrowMin;
			wynikIr.bBox[2] = BBoxIr.at<int>(2, 0);// + IRcolumnMin;
			wynikIr.bBox[3] = BBoxIr.at<int>(3, 0);// + IRrowMin;

			// Wizualizacja i utrwalenie wyników
			if (VISUAL == 1)
			{
				imshow("13.2 Klasy w IR: brzeg, szyp, wklês³e, plamki", ClassIrImg);
				if (LOGS == 1)
				{
					sprintf(outName, "%s/A13_2_ClassIR_%s.png", ouDir, ouName);
					cv::imwrite(outName, ClassIrImg);
				}
				if (LOGS == 1)
				{
					std::cout << "wynikRGB.pixNum: " << wynikRGB.pixNum << "\n";
					std::cout << "wynikRGB.cx: " << wynikRGB.cx << "\n"; //% wspó³rzêdna X œrodka masy konturu w obrazie RGB
					std::cout << "wynikRGB.cy: " << wynikRGB.cy << "\n";  //% wspó³rzêdna Y œrodka masy konturu w obrazie RGB
					std::cout << "wynikRGB.bBox[0]: " << wynikRGB.bBox[0] << "\n";  //%[minx miny maxx maxy] prostok¹t obejmuj¹cy
					//%class1; % OK - jab³ko OK(czerwone lub zielone)
					std::cout << "wynikRGB.bel1: " << wynikRGB.bel[0] << "\n";  //% procentowa zawartoœæ klasy 1
					
					//% 2 - 5 : wystarczy obraz RGB
					//%class2; % defekt - br¹zowy(plamka)
					std::cout << "wynikRGB.bel2: " << wynikRGB.bel[1]<< "\n";  //% jw - dla klasy 2
					//% class3; % defekt - szary(odcisk)
					std::cout << "wynikRGB.bel3: " << wynikRGB.bel[2]<< "\n";  //% jw
					//%class4; % defekt - ciemny(pora¿enie grzybowe)
					std::cout << "wynikRGB.bel4: " << wynikRGB.bel[3]<< "\n";  //% jw
					//%class5; % obszar szypu³ki lub wylotu worka nasiennego
					std::cout << "wynikRGB.bel5: " << wynikRGB.bel[4]<< "\n";  //% jw
					//% 6 - 7: potrzebne obrazy IR i RGB
					//%class6; % defekt - wg³êbienie ze zmian¹ koloru
					std::cout << "wynikRGB.bel6: " << wynikRGB.bel[5]<< "\n"; //% jw
					//%class7; % defekt - wg³êbienie bez zmiany koloru
					std::cout << "wynikRGB.bel7: " << wynikRGB.bel[6]<< "\n"; //%jw
					std::cout << "wynikRGB.numReg: " << wynikRGB.numReg << "\n";  //% liczba obszarów
				}
				if (LOGS == 1)
				{
					std::cout << "wynikIr.pixNum: " << wynikIr.pixNum << "\n";
					std::cout << "wynikIr.cx: " << wynikIr.cx << "\n"; // % wspó³rzêdna X œrodka masy konturu w obrazie RGB
					std::cout << "wynikIr.cy: " << wynikIr.cy << "\n"; // % wspó³rzêdna Y œrodka masy konturu w obrazie RGB
					std::cout << "wynikIr.bBox[0]: " << wynikIr.bBox[0] << "\n"; // %[minx miny maxx maxy] prostok¹t obejmuj¹cy
					//%class1; % OK - jab³ko OK(czerwone lub zielone)
					std::cout << "wynikIr.bel1: " << wynikIr.bel[0] << "\n"; // % procentowa zawartoœæ klasy 1 -
					//% 2 - 5 : wystarczy obraz RGB
					//%class2; % defekt - wgniecenie(czerwone))
					std::cout << "wynikIr.bel2: " << wynikIr.bel[1] << "\n"; //% jw - dla klasy 2
					//% class3; % defekt - plamka(czarny) na wypuk³ym
					std::cout << "wynikIr.bel3: " << wynikIr.bel[2] << "\n"; // % jw
					//%class4; % obszar przy szypu³ce(nieb)
					std::cout << "wynikIr.bel4: " << wynikIr.bel[3] << "\n"; // % jw
					//% class5; % 
					std::cout << "wynikIr.bel5: " << wynikIr.bel[4] << "\n"; // % jw
					//%class6; % wg³ebienie bez zmiany koloru 
					std::cout << "wynikIr.bel6: " << wynikIr.bel[5] << "\n"; // % jw
					//%class7; % wg³ebienie ze zmian¹ koloru
					std::cout << "wynikIr.bel6: " << wynikIr.bel[6] << "\n"; // % jw
				}
			}

			//
			//
			// 13.3 Scalenie i konwersja wyników klasyfikacji 2D i 3D na dane wyjsciowe
			// Obrazy IR i RGB nie s¹ identyczne, dlatego zasadniczo klasy wykryte w RGB
			// s¹ modyfikowane(procentowo) przez wyniki w IR

			cv::Mat classMap;
			ROD::ResultOD retvalue = ROD::ScalWyniki(mcy, mcx, uparam, MaskImg, ClassImg, ClassIrImg, classMap, 
				wynikRGB, wynikIr, result);
			// w zasadzie zwraca referencjê do zmodyfikowanego obiektu wynikRGB

			// Wizualizacja i utrwalenie wyników
			if (VISUAL == 1)
			{
				imshow("13.3 Klasy w scalonym obrazie", retvalue.classImage);
				if (LOGS == 1)
				{
					sprintf(outName, "%s/A13_3_Class_%s.png", ouDir, ouName);
					cv::imwrite(outName, retvalue.classImage);
				}

			}
			// Koniec pomiaru czasu dla kroku 13
			end = time(0);
			elapsed = (double)(clock() - t0) / CLOCKS_PER_SEC;
			cout << "Czas kroku 13: " << endl;
			cout << elapsed << " ms" << endl;
			cout << end - start << " s" << endl;
			
			if (LOGS == 1)
			{
					std::cout << "wynik.pixNum: " << retvalue.pixNum << "\n";
					std::cout << "wynik.cx: " << retvalue.cx << "\n"; //% wspó³rzêdna X œrodka masy konturu w obrazie RGB
					std::cout << "wynik.cy: " << retvalue.cy << "\n";  //% wspó³rzêdna Y œrodka masy konturu w obrazie RGB
					std::cout << "wynik.bBox[0]: " << retvalue.bBox[0] << "\n";  //%[minx miny maxx maxy] prostok¹t obejmuj¹cy
					//%class1; % OK - jab³ko OK(czerwone lub zielone)
					std::cout << "wynik.bel1: " << retvalue.bel[0] << "\n";  //% procentowa zawartoœæ klasy 1
					
					//% 2 - 5 : wystarczy obraz RGB
					//%class2; % defekt - br¹zowy(plamka)
					std::cout << "wynik.bel2: " << retvalue.bel[1]<< "\n";  //% jw - dla klasy 2
					//% class3; % defekt - szary(odcisk)
					std::cout << "wynik.bel3: " << retvalue.bel[2]<< "\n";  //% jw
					//%class4; % defekt - ciemny(pora¿enie grzybowe)
					std::cout << "wynik.bel4: " << retvalue.bel[3]<< "\n";  //% jw
					//%class5; % obszar szypu³ki lub wylotu worka nasiennego
					std::cout << "wynik.bel5: " << retvalue.bel[4]<< "\n";  //% jw
					//% 6 - 7: potrzebne obrazy IR i RGB
					//%class6; % defekt - wg³êbienie ze zmian¹ koloru
					std::cout << "wynik.bel6: " << retvalue.bel[5]<< "\n"; //% jw
					//%class7; % defekt - wg³êbienie bez zmiany koloru
					std::cout << "wynik.bel7: " << retvalue.bel[6]<< "\n"; //%jw
					std::cout << "wynik.numReg: " << retvalue.numReg << "\n";  //% liczba obszarów
			}
			cout<<"Koniec AnalizaJablka()"<<endl;

			//cv::waitKey();
			//cv::destroyAllWindows();

			return retvalue;//wynik;

		}
		

		void putStreamOUT(ResultOD *sOUT)
		{
			this->streamOUT =  sOUT;
		}
	}; // Koniec definicji klasy Analysis

	
}

#endif
