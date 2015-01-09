#ifndef PARAMETERS
#define PARAMETERS

namespace ROD {
	const double defaultThresh = 0.2; // ROD
	const float defaultRelDetThresh = 0.11f; // STEM
	const int defaultBackgroundThresh = 20; // STEM
	const float defaultselStemPR = 0.8f; // STEM

	class Parameters{
		// Wewnêtrzne parametry ustawiane przez in¿yniera serwisowego
	public:
        int lowThresh; // nieistotna si³a krawêdzi, np. 8    
		double edgeThresh; // adaptacyjny próg pocieniania krawêdzi
        double cornerMax; // for ROI detection in IR image
        double cornerMin;
        double redColorThresh;
        double intensityThresh;
		int directions; // podzia³ k¹ta pe³nego

		// Dla STEM
		float relDetThresh; // 
	    int backgroundThresh; // 
		float selStemPairRatio; //
			
	public: 
		Parameters(int low = 8, double th = 0.2, double cmax= 100.0, double cmin = 50.0, double red = 48.0, 
			double intens = 80.0, int dirs = 720, float rdt = defaultRelDetThresh, int bat = defaultBackgroundThresh,
			float sspr=defaultselStemPR )
		{
			// ROD
			this->lowThresh = 8;
			this->edgeThresh = th; // edge strength threshold
            this->cornerMax = cmax;
            this->cornerMin = cmin;
            this->redColorThresh = red;
            this->intensityThresh = intens;
			this->directions = dirs;
			// STEM
			this->relDetThresh = rdt;
			this->backgroundThresh= bat;
			this->selStemPairRatio = sspr;
			
		}
		
	};
}

	class SFSParameters{
		// Wewnêtrzne parametry ustawiane przez in¿yniera serwisowego
		// dla biblioteki ananlizy 3D metod¹ SFS
	public:
            cv::Point3d light;
			int crop;
			int gaussMat;
			double gaussSig; 
			int iter;
			int thresholdArea;
			int numClasses; 
			
	public: 
		SFSParameters(cv::Point3d lightPar, int cropPar=1, int gaussMatPar=11, double gaussSigPar=11.0,
			int iterPar=10, int thresholdAreaPar=20, int numClassesPar=8)
		{
			this->light = lightPar;
			this->crop = cropPar;
			this->gaussMat = gaussMatPar;
			this->gaussSig = gaussSigPar; 
			this->iter = iterPar;
			this->thresholdArea = thresholdAreaPar;
			this->numClasses = numClassesPar; 
			
		}
		
	};
#endif
