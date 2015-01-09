#ifndef RESULT_OD
#define RESULT_OD

//#include <cv.h>
#include <opencv/cv.h>
//#include <opencv/ml.h>
//#include <opencv2/core/core.hpp>    

namespace ROD {
	struct ResultOD{
	long pixNum;  // obszar jabłka w obrazie RGB (w pikselach)
	int cx; // współrzędna X środka masy konturu w obrazie RGB (ROD)
    int cy; // współrzędna Y środka masy konturu w obrazie RGB (ROD)
	int radius; // promień obszaru jabłka w RGB
    int bBox[4]; // [minx miny maxx maxy] prostokąt obejmujący
	int jestOgon; // szypułka odstająca = 1
	int jestSzyp; // szypułka wewnętrzna = 1
	int ogonCx; // położenie nasady szypułki; wsp. X
	int ogonCy; // - " - , wsp. Y
	
	double bel[7]; // procentowa zawartość pikseli klas defektów
	// Klasa 1;  OK - jabłko OK (czerwone lub zielone)
    // Klasy defektów 2-5 : wystarczy obraz RGB
    // Klasa 2;  defekt - brązowy (plamka)
    // Klasa 3;  defekt - szary (odcisk)
    // Klasa 4;  defekt - ciemny (porażenie grzybowe)
	// Klasa 5;  obszar szypułki lub wylotu worka nasiennego
    // Klasy 6-7: potrzebne obrazy IR i RGB
    // Klasa 6;  defekt - wgłębienie ze zmianą koloru
	// Klasa 7;  defekt - wgłębienie bez zmiany koloru
            
    int numReg; // liczba obszarów
	cv::Mat regions;// macierz opisów obszarów [numReg,6]
    int *classReg; // wektor identyfikatorów klas dla obszarów [numReg]
    ::cv::Mat classImage; // poklasyfikowane obszary obrazu jabłka - obraz RGB
	};

	struct ResultODline{
		ResultOD *resultL;
	};

}
#endif
