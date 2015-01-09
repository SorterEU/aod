// Plik rod_KonturIR.cpp

	//
	// Funkcja pomocnicza 7: KonturIR()
	//

// Funkcja zwraca wynik w postaci poniższych zmiennych globalnych:
// TO DO: to muszą być parametry funkcji
//	int Cirx, Ciry; // środek masy konturu
//	::cv::Mat IrKontur = ::cv::Mat::zeros(720, 2, CV_64FC1); // wektor współrzędnych punktów konturu
//	::cv::Mat IrKontur1D = ::cv::Mat::zeros(720,1, CV_64FC1); // odległości punktów konturu od środka masy
//	::cv::Mat IrGrad1D = ::cv::Mat::zeros(720,1, CV_64FC1); // gradient funkcji odległości

	void KonturIR(double thresh1, double thresh2, int DIRS, int ICx, int ICy, int minRad, ::cv::Mat &MImg,
		int &Cirx, int &Ciry, ::cv::Mat &IrKontur, ::cv::Mat &IrKontur1D, ::cv::Mat &IrGrad1D )
	{
	// Wyznacza kontur obiektu (2D, 1D, 1DGrad) w obrazie krawędziowym IR oraz
	// jego środek masy, (Cirx, Ciry)
	// DIRS : liczba badanych kierunków = elementów konturu
	int DIRS1 = DIRS -1;
	int mx, my, MINRADIUS;
	double maxValue;
	double delrad = 2.0 * PI / DIRS;
	double alfa, x, y, dx, dy, dr;
	int yr0, yr1, yr2, xr0, xr1, xr2, startInd, jestN, iend;
	double p11, p12, p13;
	double r, r0, r1, i0, meanRadius;
	int radElNum; 

	//mx=MImg.rows; my=MImg.cols;
	my=MImg.rows; mx=MImg.cols;
	cv::minMaxLoc(MImg,0,&maxValue, 0, 0, cv::Mat() );
	
	//% Tymczasowy środek
	Cirx = ICx; Ciry = ICy;
	MINRADIUS = minRad; // 51;
	
	//%CONSTANTS
	const double MINVALUE = thresh1 * maxValue; //% e.g. 0.2
	const double MINREDVALUE = thresh2 * maxValue; //% e.g 0.3
	
	//% Stałe pomocnicze
	int HalfI1;
	const int HalfX = round(mx / 2);
	//%QuatX = mx / 4;
	//%Quat3X = QuatX + HalfX;
	const int HalfY = round(my / 2);
	

	if (HalfY > HalfX)
		HalfI1 = HalfY - 1;
	else
		HalfI1 = HalfX - 1;
	
	//% Alokacja pamięci pomocniczej
	::cv::Mat jestKontur = ::cv::Mat::zeros(DIRS,1, CV_8UC1); // czy jest punkt?
	::cv::Mat sinalfa = ::cv::Mat::zeros(DIRS,1, CV_64FC1); // wartości funkcji sin
	::cv::Mat cosalfa = ::cv::Mat::zeros(DIRS,1, CV_64FC1); // wartości funkcji cos

	//%
	alfa = -PI; 
	meanRadius = 0.0;
	radElNum = 0;
	for (int i=0; i<DIRS; i++)
	{
		alfa += delrad;
		sinalfa.at<double>(i,0) = sin(alfa);
		cosalfa.at<double>(i,0) = cos(alfa);
		for (int r=MINRADIUS; r<=HalfI1; r++)
		{
			x = Cirx + (r+1) * cosalfa.at<double>(i,0);
			y = -((r+1) * sinalfa.at<double>(i,0) - Ciry);
			yr0 = round(y);
			xr0 = round(x);
			x = Cirx + r * cosalfa.at<double>(i,0);
			y = -(r * sinalfa.at<double>(i,0) - Ciry);
			yr1 = round(y);
			xr1 = round(x);
			x = Cirx + (r-1) * cosalfa.at<double>(i,0);
			y = -((r-1) * sinalfa.at<double>(i,0) - Ciry);
			yr2 = round(y);
			xr2 = round(x);
        
			if ((xr0 > 0) && (xr0 <= mx))
			{
				if ((yr0 >0) && (yr0 <= my))
                {
					IrKontur.at<double>(i,0) = xr0;
					IrKontur.at<double>(i,1) = yr0;
				//%y = - (tgalfa * (i - Cirx) - Ciry); % use image coordinates but change the
				//% sign of the result into memory index
					p11 = MImg.at<uchar>(yr0-1,xr0-1);
					p12 = MImg.at<uchar>(yr1-1,xr1-1);
					p13 = MImg.at<uchar>(yr2-1,xr2-1);
					if ( (p11 < MINVALUE) && ( p12 < MINREDVALUE) && (p13 < MINREDVALUE) )
					{
						if (r < (int)(0.9 * meanRadius))
							meanRadius = meanRadius; // nic nie rób
						else {
							IrKontur.at<double>(i,0) = xr1;
							IrKontur.at<double>(i,1) = yr1;
							jestKontur.at<uchar>(i,0) = 1;

							
							meanRadius = meanRadius * radElNum + r;
							radElNum ++;
							meanRadius = meanRadius / radElNum;
							
							break;
						}
					}
				}
			}
		}
	}
	//% Aproksymuj brakujące pomiary
	startInd = -1;
	for (int i=0; i<DIRS1;i++)
		if (jestKontur.at<uchar>(i,0) == 1)
		{
			startInd = i;
			dx = IrKontur.at<double>(startInd, 0) - Cirx;
			dy = -(IrKontur.at<double>(startInd, 1) - Ciry);
			r0 = sqrt(dx*dx + dy*dy);
			break;
		}
	
	if (startInd > -1)
	{
		r = r0;
		i0 = startInd+1;
		for (int i= i0; i<DIRS; i++) //i0: DIRS 
		{
			if (jestKontur.at<uchar>(i,0) == 0)
			{
				//% znajdź następny
				jestN=0;
				for (int j=i;j<DIRS;j++) //i:DIRS
					if (jestKontur.at<uchar>(i,0) == 1)
					{
						jestN=1;
						dx = IrKontur.at<double>(j, 0) - Cirx;
						dy = -(IrKontur.at<double>(j, 1) - Ciry);
						r1 = sqrt(dx*dx + dy*dy); 
						dr = (r1 - r) /(j-i);
						break;
					}
				//%aproksymuj
				if (jestN==1)
					r= r+dr;
				x = Cirx + r * cosalfa.at<double>(i,0);
				y = -(r * sinalfa.at<double>(i,0) - Ciry);
				yr0 = round(y);
				xr0 = round(x);
				IrKontur.at<double>(i,0) = xr0;
				IrKontur.at<double>(i,1) = yr0;
				jestKontur.at<uchar>(i,0) = 1;
			}
			else
			{
				//%new r
				dx = IrKontur.at<double>(i,0) - Cirx;
				dy = -(IrKontur.at<double>(i,1) - Ciry);
				r = sqrt(dx*dx + dy*dy);
			}
		}
			
		if (startInd > 1) //startInd > 1
		{
			//%aproksymuj początek
			r1= r;
			dr = (r0 - r1) /startInd;
			iend = startInd - 1;
			for (int i=0; i<iend; i++)
			{
				r = r + dr;
				x = Cirx + r * cosalfa.at<double>(i,0);
				y = -(r * sinalfa.at<double>(i,0) - Ciry);
				yr0 = round(y);
				xr0 = round(x);
				IrKontur.at<double>(i,0) = double(xr0);
				IrKontur.at<double>(i,1) = double(yr0);
				jestKontur.at<uchar>(i,0) = 1;
			}
		}
	}
	//% Oblicz srodek masy konturu
	Cirx = round(cv::mean(IrKontur.col(0)).val[0]);
	Ciry =round(cv::mean(IrKontur.col(1)).val[0]);

	// cout<<"Cirx, Ciry:"<< Cirx<< ", " << Ciry<< endl;

	double Cxd = (double)Cirx;
	double Cyd = (double)Ciry;
	//%wyznacz funkcję 1D odległości punktów konturu od środka masy
	for (int i=0;i<DIRS;i++)
	{
		if (i < 10) {
			dx = IrKontur.at<double>(i, 0) - Cxd;
			dy = -(IrKontur.at<double>(i, 1) - Cyd);
		}
		else {
			dx = IrKontur.at<double>(i, 0) - Cxd;
			dy = -(IrKontur.at<double>(i, 1) - Cyd);
		}
		if (i < DIRS1)
    		IrKontur1D.at<double>(i,0) = sqrt(dx*dx + dy*dy);
		else 
			IrKontur1D.at<double>(i,0) = sqrt(dx*dx + dy*dy);

	}

	//%wyznacz gradient funkcji 1D odległości konturu
	IrGrad1D.at<double>(0,0) = 0.5 * (IrKontur1D.at<double>(1,0) - IrKontur1D.at<double>(DIRS-1,0));
	IrGrad1D.at<double>(DIRS-1,0) = 0.5 * (IrKontur1D.at<double>(0,0) - IrKontur1D.at<double>(DIRS1-1,0));
	for (int i=1;i<DIRS1;i++)
		IrGrad1D.at<double>(i,0) = 0.5 * (IrKontur1D.at<double>(i+1,0) - IrKontur1D.at<double>(i-1,0));

	//std::cout<<"IrGrad1D: "<<IrGrad1D<<"\n";
	jestKontur.release();
	sinalfa.release();
	cosalfa.release();
	}
