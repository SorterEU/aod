// Aplikacja CAppSorter
// Funkcja Main(): punkt wejścia do programu - aplikacji konsolowej z użyciem biblioteki OpenCV
// Autor: Włodzimierz Kasprzak
// Ostatnia modyfikacja: 18-11-2014
// Obszary nazw:
// - ROD : receptor agenta OD - do detekcji i oceny defektów powierzchni jabłka - 
//		- - klasa główna to "Analysis" - definicja w pliku Analysis.h
// - SFS : analiza 3D (Autorka: Karolina Przerwa)- klasa główna to "CALIBRATOR" 
//	    - - opis poarametrów w pliku Calibrator.HPP
// - AW : analiza 2D obrazu IR i Mono (uzyskanego z RGB) (Autor: Artur Wilkowski)
// 

//#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

// dla ROD
#include "ROD/Parameters.h"
#include "ROD/Properties.h"
#include "ROD/UserParameters.h"
#include "ROD/ResultOD.h"
#include "ROD/Analysis.hpp"

// dla SFS 
#include "SFS/Resolver.hpp"
#include "SFS/ATDA.hpp"
#include "SFS/Calibrator.hpp"

using namespace std;

///
// Funkcja main() - punkt wejścia do programu
///

int main(int argc, char * argv[])
{
	// Uwaga: dane wejściowe są pobierane ze strumienia wejściowego streamIN - w docelowej implementacji
	// ale w tym programie w trybie off-line są pobierane z plików w katalogu

	// Praca z programem:
	// Należy utworzyć i zainicjalizować obiekty klasy Analysis i Calibrator
	// a następnie wywołać metody Analysis.OcenaDefektow() - dla każdego widoku jabłka - i
	// Analysis.DefektyJablka(): dla integracji wyników dla kilku widoków jabłek w liniach
	
	// Przełączniki pomocnicze: dla wizualizacji, monitorowania i "utrwalania" wyników 
	const int VIS = 0;  // wizualizacja obrazów
	const int MONIT = 1; // debug - tekst
	const int LOG = 0; // utrwalenie wyników

	// Pomiar czasu dla oceny wydajności kroków programu
	double t0, elapsed;
	time_t start, end;

	// KROK 0:
	// INICJALIZACJA obiektów

	// Obiekt z własnościami stanowiska
	ROD::Properties pr = ROD::Properties(10, 9); // liczba widoków i linii (różnych jabłek); domyślnie =(8, 3)
	// Obiekt z parametrami analizy obrazu
	ROD::Parameters pa = ROD::Parameters(16, 0.2, 0.5, 0.20, 50, 20, 720); // low edge, edge threshold, corner max, corner min
		// red color thresh min, intensity thresh min, direction's number
	// Obiekt z parametrami definiowanymi przez użytkownika
	ROD::UserParameters	upa = ROD::UserParameters(1); // typ jabłka: 1- czerwony
	
	// Obiekt z parametrami biblioteki SFS 3D (ATDA)
	SFSParameters sfspa= SFSParameters(cv::Point3d(0.001, 0.001, 1.0), 1, 11, 11.0, 10, 20, 8);

	// "paczka" danych wej�ciowych
	cv::Mat  sIN = cv::Mat();
	// generowany wynik analizy "paczki" obraz�w
	ROD::ResultOD sOUT = ROD::ResultOD();

	// obiekty potrzebne do analizy obraz�w
	ROD::Analysis *rodobj; // obiekt g��wny

	ROD::ResultODline *storedResults; // pomocnicza struktura wynikow cz�ciowych

	// Katalog WE: tryb off-line
	char inDir[] = "C:/SORTER/img/Database/01_Diffuse";
//	char inDir[] = "/home/jfigat/Projekty/SORTER/Database/01_Diffuse";
//	char inDir[] = "D:/workspace/SORTER_Analysis/img/Database/01_Diffuse";
	
	//Katalog WY: tryb off-line
	char ouDir[] = "log2";
	// W trybie on-line wyniki przekazuje się do bufora wyjściowego streamOUT

	char *in = &inDir[0];
	char *ou = &ouDir[0];

	std::cout<< argc << std::endl;

	if( argc < 3) // jeśli w wywołaniu brak argumentów podających pierwszy katalog 
    {
     std::cout <<" argumenty: katalog_in katalog_log" << std::endl;
	 std::cout <<" ustaw katalogi domyślne" << std::endl;
	}
	else
	{
		in = argv[1];
	}
	
	if( argc < 4) // jeśli w wywołaniu brak argumentów podających drugi katalog 
    {
		cout <<" argumenty: katalog_in katalog_log" << endl;
		cout <<" ustaw katalogi domyślne" << endl;
	}
	else
	{
		ou = argv[2];
	}


	// KROK 1: 
	// Analiza "paczki" obrazów
	//
	t0=clock();		// zliczanie czasu
	start=time(0);
	// Inicjalizacja obiektów: klasy Analysis i Calibrator
	// oraz wywołanie funkcji analizy obrazów:
	// - OcenaDefektów - analiza pojedynczego widoku
	// - DefektyJablka - integracja wyników wielu widoków jednego jabłka
	try
	{
		// Utwórz obiekt klasy Analysis zdefiniowanej w przestrzeni nazw ROD
		rodobj = new ROD::Analysis(&sIN, pr, pa, upa, &sOUT, VIS, MONIT, LOG);

		// Inicjalizuj obiekt dla analizy 3D metodą sfs
		rodobj->sfs3D = new Calibrator(sfspa, "SFS/mlp.xml", VIS, MONIT, LOG);  // obiekt główny biblioteki SFS

		// Alokuj pamięć wyników cząstkowych
		storedResults = new ROD::ResultODline[pr.getNumViews()];
		for (int t=0; t < pr.getNumViews(); t++)  // dla ka�dego widoku
		{
			storedResults[t].resultL = new ROD::ResultOD[pr.getNumLines()]; // widoki w jednej linii
		}

		// Teraz uruchamiamy analizę obrazów
		// Teoretycznie nieskończona pętla wykonywana dla każdej paczki (N linii x n widoków)
		ROD::ResultOD *results = new ROD::ResultOD[pr.getNumLines()];
		//do (true ) // W trybie off-line: wykomentowane
		{
		// Blok "główny"
		// 1) Wywołanie funkcji detekcji i oceny defektów (metoda w klasie ROD::Analysis)
			rodobj->OcenaDefektow(in, ou, storedResults); // wynik zwracany jest w storedResults 
        
		// 2) Integruj wyniki widoków jednego jabłka        
			rodobj->DefektyJablka(storedResults, results); // Funkcję implementuje M.Stefańczyk
			
		// 3) Przekaż wyniki do strumienia wyjściowego: 
			rodobj->putStreamOUT(results);
		} // koniec bloku głównego

		// zwalnianie pamięci dynamicznej
		delete [] results;
		for (int t=0; t < pr.getNumViews(); t++)  // dla każdego widoku
		{
			delete [] storedResults[t].resultL; 
		}
		delete [] storedResults;
		delete rodobj;

	}
	catch(cv::Exception &e){
	    cout<<"Exception"<<endl;
		cerr << e.what() <<endl;
	}
	catch(std::exception &e){
		cerr << e.what() <<endl;
	}
	catch(char const * e){
		cerr << e <<endl;
	}
	



/*
	// Inicjalizacja 
	//rozpoznawanie - przyklad użycia 
	try{
	Calibrator *solution = new Calibrator(cv::Point3d(0.001, 0.001, 1.0), 
		1, 11, 11, 10, 20, 1, 8, "SFS/mlp.xml");
	vector<StatsRoi> rois; // = solution->solution(argv[1]);
	rois = solution->solution("01_0.png");
	delete solution;
	}
	catch(ATDAException &e){
		cerr << e.what() <<endl;
	}
	catch(cv::Exception &e){
		cerr << e.what() <<endl;
	}
	catch(std::exception &e){
		cerr << e.what() <<endl;
	}
	catch(char const * e){
		cerr << e <<endl;
	}
*/

	/* //Uczenie
	try{
	Calibrator(cv::Point3d(0.001, 0.001, 1.0), 
		1, 11, 11, 10, 20, 0, 8, L"F:/SORTER_DB/Database/01_Diffuse", 
		"regulyv1.csv","parametryv1.xml", "klasyv1.xml", "zbioryv1.csv",
		"mlpv1.xml");
	//solution->solution("F:/SORTER_DB/Examples/9.bmp");

	}
	catch( cv::Exception &e){cerr<<e.what();}
			catch( ATDAException &e){cerr<<e.what();}
	catch (std::exception &e){cerr<<e.what();}
	*/
	
	end=time(0);
	elapsed= (double)(clock() - t0) ;

	
	cout<< "t0= " << t0 << " t1= " << clock() << endl;
	cout << "Czas wykonania main():" << endl;
	cout << elapsed<< " ms" <<  endl;
	cout << end-start << " s" <<  endl;
	
	cv::waitKey(0);	
	return 0;
} 
// Koniec pliku main.cpp

