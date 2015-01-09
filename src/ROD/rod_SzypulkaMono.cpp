// Funkcja SzypulkaMono()
// i jej podfunkcje
// GenerujAktWzorzec(), Matching(), AngleDifftoPihalf()

	//
	//
	// Funkcja pomocnicza 9: GenerujAktWzorzec()
	//

	cv::Mat roiSzypMono;
	double katSzyp;
	cv::Mat ocenaSzypulki;
	cv::Mat pozycjaSzypulki;
	cv::Mat MaskaMonoSzyp;
	int jestSzypROD; // wewnętrzna szypułka wykrywana metodą ROD::SzypulkaMono
	double ocena, delA;


	/*void*/cv::Mat GenerujAktWzorzec(int sy, int sx, double cosaI, double sinaI,  cv::Mat &Wzor/*, cv::Mat &aWzor*/)
	{
		
		double wCx, wCy, wC1x, wC1y;
		double smax, Tx, Ty;
		int swy;
		double xn_, yn_;
		int xni, yni;
		cv::Mat aWzor;
		//% środek wzorca - oryginał
		wCx = (sx + 1)/ 2; //% sx jest z założenia nieparzyste
		wCy = 1;
		//% środek docelowego wzorca - o podwójnym rozmiarze
		smax = sx;//20
		wC1x = wCx * 2;//21
		wC1y = wC1x;//21
		Tx = wC1x - wCx;//10.5
		Ty = wC1y - wCy;//20
		//% alokacja dla wyniku
		swy = smax*2 + 1;//41
		aWzor = cv::Mat::zeros(swy, swy, CV_64FC1);
		
		for (int y = 0; y< sy;y++)
			for (int x = 0;x<sx;x++)
			{
				if (Wzor.at<double>(y,x) != 0)//~=0
				{
					//% nowe współrzędne punktu
					xn_ = (x - wCx) * cosaI - (-(y - wCy)) * sinaI;
					yn_ = (x - wCx) * sinaI + (-(y - wCy)) * cosaI;
					yn_ = - yn_;
					xn_ = xn_ + wCx + Tx;
					yn_ = yn_ + wCy + Ty;
		            xni = round(xn_)-1;
		            yni = round(yn_)-1;
					//std::cout << "TEST: yni, xni " << yni << ";" << xni << "\n";
		            if ((xni>=0) && (yni >= 0))
		            {
		            	//% set nonzero pixel
		            	aWzor.at<double>(yni/*-1*/, xni/*-1*/) = Wzor.at<double>(y,x);
		            }
					//std::cout << "TEST: yni, xni " << yni << ";" << xni << "\n";
				}
			}
			
		return aWzor;
	}

	//
	//
	// Funkcja pomocnicza dla 9: 9a: AngleDiffToPihalf()
	//
	double AngleDiffToPihalf(double alfa_,double edge_)
	{
		const double PIHALF = 0.5 * ROD::PI;
		delA = edge_;
		//% Zakładamy, że alfa jest juz zrzutowane
		//% Teraz tylko rzut "edge" na podprzedział <-pi/2, pi2>
		if (delA <= -PIHALF)
		    delA = delA + ROD::PI;
		else if (delA > PIHALF)
			delA = delA - PI;
		delA = abs(alfa_ - delA);//%  - PIHALF;
		//% delA powinno być w okolicy zera
		return delA;
	}

	//
	//
	// Funkcja pomocnicza dla 9: 8b: Matching()
	//
	double Matching(double sy_, double sx_, double alfa_,cv::Mat Dimg_,cv::Mat Mimg_,cv::Mat MonoImg_,
		cv::Mat aWzor_,double AThresh_)
	{
	ocena = 0;
	double delA;
	
	for (int i=0;i<sy_;i++)
	    for (int j=0;j<sx_;j++)
	        if (aWzor_.at<double>(i,j) != 0)
	            if ((Mimg_.at<uchar>(i,j) > 0) && (MonoImg_.at<uchar>(i,j) < 120))
	            {
	                delA = ROD::AngleDiffToPihalf(alfa_, (double)Dimg_.at<float>(i, j));
	                // krawędź
	                if ((delA > - AThresh_) && (delA < AThresh_))// % zgodność kąta
	                {
	                	// dodaj siłę krawędzi
	                	ocena = ocena + double(Mimg_.at<uchar>(i, j)) * aWzor_.at<double>(i,j);
	                }
	                //% else
	                //% ocena = ocena - 0.2*Mimg_.at<double>(i,j); // niezgodna krawędź
	            }
	return ocena;

	}


	//
	//
	// Funkcja pomocnicza 10: SzypulkaMono()
	//
	
	void SzypulkaMono(int Cx_, int Cy_, cv::Mat &Kontur_, cv::Mat &Kontur1D_, cv::Mat &MonoImg_, 
		cv::Mat &EMImg_, cv::Mat &EDImg_, cv::Mat &Wycinek_,int Radius, double AngleThresh_,double ScoreThresh_)
	{
		int DIRS = Wycinek_.at<int>(2,0); // 
		
		int MINRAD =  Wycinek_.at<int>(1,0) - Wycinek_.at<int>(2,0);
		if (MINRAD < 0) MINRAD = -MINRAD;
		if ((MINRAD + 1) >= Wycinek_.at<int>(2,0)) // przeszukiwanie pełne
			MINRAD = 0;
		else MINRAD = Radius - 40; // przeszukiwanie przy ogonku zewnętrznym
		if (MINRAD < 0) MINRAD = 0;

		ROD::jestSzypROD = 1;
		
		double ScoreThresh2 = 0.5 * ScoreThresh_;
		int sy =MonoImg_.rows;
		int sx =MonoImg_.cols;
		int i,j;
		
		double PIHALF = ROD::PI / 2.0;
		double delrad, alfaI, alfa;
		double xend, yend, dist,sinalfaI, cosalfaI;
		int aktOcena, ocenaDod;
		int r, r0, yr0, xr0,x0, y0;
		double x, y;
		double ocena;
		cv::Mat ANGLES;
		cv::Mat Dimg, Mimg, Mono;
	
		// Wzorce
		// 1: dwie krawędzie - szypułka
		// zdefiniuj worzec podstawowy
		cv::Mat Wzor1 = cv::Mat::zeros(20, 20,CV_64FC1);
		cv::Mat Wzor2 = cv::Mat::zeros(5, 20,CV_64FC1);
		cv::Mat aWzor1, aWzor2;

		MaskaMonoSzyp = cv::Mat::zeros(sy,sx,CV_64FC1);//zeros(sy, sx);
		
		for (i=0;i<20;i++)
		{
		    Wzor1.at<double>( i, 3-1)=  1;
		    Wzor1.at<double>( i, 4-1) = 2;
		    Wzor1.at<double>( i, 5-1) = 3; 
			Wzor1.at<double>( i, 6-1) = 3;
		    Wzor1.at<double>( i, 7-1) = 2;
		    Wzor1.at<double>( i, 8-1) = 1;
		    Wzor1.at<double>( i, 9-1) = 0;
		    Wzor1.at<double>( i, 10-1) = -1;
		    Wzor1.at<double>( i, 11-1) = -1;
		    Wzor1.at<double>( i, 12-1) = 0;
		    Wzor1.at<double>( i, 13-1) = 1;
		    Wzor1.at<double>( i, 14-1) = 2;
		    Wzor1.at<double>( i, 15-1) = 3;
			Wzor1.at<double>( i, 16-1) = 3;
		    Wzor1.at<double>( i, 17-1) = 2;
		    Wzor1.at<double>( i, 18-1) = 1;
		}
		for (i=0;i<=2;i++)
		{
			for(j=0;j<20;j++)
			{
				if(j>=2&&j<=18)
				{
					if(i!=1){Wzor2.at<double>(i,j)=1;}
					else if (i==1)Wzor2.at<double>(i,j)=2;
				}
				if(j==0||j==20)
				{
					if(i!=1){Wzor2.at<double>(i+2,j)=1;}
					else if(i==1){Wzor2.at<double>(i+2,j)=2;}
				}
				if(j==1||j==19)
				{
					if(i!=1){Wzor2.at<double>(i+1,j)=1;}
					else if(i==1){Wzor2.at<double>(i+1,j)=2;}
				}
			}
		}

		// Inicjalizacja pętli
		// Szypulka = zeros(DIRS); // sumaryczna siła krawędzi zgodnych z kierunkiem
		ocenaSzypulki = cv::Mat::zeros(DIRS,1,CV_64FC1);//  maksymalna ocena dla kierunku
		cv::Mat pozycjaSzypulki_ = cv::Mat::zeros(DIRS,2,CV_32SC1); //%

		delrad = 2.0 * ROD::PI / DIRS;

		ANGLES = cv::Mat::zeros(DIRS,1,CV_64FC1);

		double temp;
	   // Sprawdzenie właściwej wartości danych w zmiennej Wycinek
		//if(Wycinek_.at<int>(0,0) > Wycinek_.at<int>(1,0)) { 
			// std::cout<<"ERROR Wycinek_ \n"; zamian kolejności obu elementów 
//			temp = Wycinek_.at<int>(0,0);
	//		Wycinek_.at<int>(0,0)=Wycinek_.at<int>(1,0);
		//	Wycinek_.at<int>(1,0)=temp;
		//}

		alfaI = - ROD::PI + (Wycinek_.at<int>(0,0)) * delrad;
		
		//cout << "Ir Kontur1D: ";
		//for (i=0; i< 720; i++) 
		//	cout<< 	Kontur1D_.at<double>(i,0) << "; " ;
		//cout << endl;

		i = Wycinek_.at<int>(0,0);
		for (int i0 = 0; i0 < DIRS; i0++)
		{
			alfaI = -ROD::PI + i * delrad;

		    xend = Kontur_.at<double>(i, 0);
		    yend = Kontur_.at<double>(i, 1);
		    dist = Kontur1D_.at<double>(i,0); // sqrt(dx*dx + dy*dy);
		    aktOcena = 0;

		    ANGLES.at<double>(i,0) = alfaI;
		    sinalfaI = sin(alfaI);
		    cosalfaI = cos(alfaI);
		    // rzut kierunku na podprzedział <-pi/2, pi/2>
		    alfa = alfaI;
		    if (alfa <= - PIHALF)
		        alfa = alfa + ROD::PI;
		    else if (alfa > PIHALF)
		    	alfa = alfa - ROD::PI;
		    // Obrócony wzorzec
		    aWzor1 = cv::Mat::zeros(41,41,CV_64FC1);
		    aWzor2 = cv::Mat::zeros(41,41,CV_64FC1);
		    aWzor1 = ROD::GenerujAktWzorzec(20, 20, cosalfaI, sinalfaI, Wzor1);//, aWzor1);
			aWzor2 = ROD::GenerujAktWzorzec(5, 20, cosalfaI, sinalfaI, Wzor2);//, aWzor2);

		    // Pętla wzdłuż prostej
		    r0 = int(dist) - 20; // gdyż stosujemy najpierw wzorzec 1
		    for (r= r0 ; r>=MINRAD; r-=2)
		    {
		    	// położenie środka sprawdzanego obszaru
		    	x = Cx_ + r * cosalfaI;
		    	y = -(r * sinalfaI - Cy_);
		    	yr0 = round(y);
		    	xr0 = round(x);
		    	// wytnij aktualny fragment obrazu
		   
		    	Dimg = EDImg_(cv::Rect(xr0-20-1,yr0-20-1, 41, 41)).clone();
		    	Mimg = EMImg_(cv::Rect(xr0-20-1,yr0-20-1, 41, 41)).clone();
		    	Mono = MonoImg_(cv::Rect(xr0-20-1,yr0-20-1, 41, 41)).clone();

		    	ocena = ROD::Matching(41, 41, alfa, Dimg, Mimg, Mono, aWzor1, AngleThresh_);
		    	Dimg.release();
		    	Mimg.release();
		    	Mono.release();

		    	// Czy maksimum oceny dla kierunku?
		    	if (int(ocena) > aktOcena)
		    	{
		    		pozycjaSzypulki_.at<int>(i, 0) = xr0;
		    		pozycjaSzypulki_.at<int>(i, 1) = yr0;
		    		ocenaSzypulki.at<double>(i,0) = ocena;
		    		aktOcena = ocena;
		    	}
				
		    }

		    ocenaSzypulki.at<double>(i,0) = aktOcena;
		    
			// Teraz ewentualnie dopasuj drugi wzorzec
		    
			if (aktOcena > ScoreThresh_)//  coś zostało znalezione
		    {
		    	x0 = pozycjaSzypulki_.at<int>(i, 0);
		    	y0 = pozycjaSzypulki_.at<int>(i, 1);
		    	ocenaDod = 0;
		    	for (int dx=-1; dx<=1;dx++)
		    		for (int dy=-1; dy<=1;dy++)
		    		{
		    			x = x0 + dx;
		    			y = y0 + dy;
		    			// wytnij aktualny fragment obrazu
		    			Dimg = EDImg_(cv::Rect(x-20-1,y-20-1, 41, 41)).clone();
		    			Mimg = EMImg_(cv::Rect(x-20-1,y-20-1, 41, 41)).clone();
		    			Mono = MonoImg_(cv::Rect(x-20-1,y-20-1, 41, 41)).clone();
		    			ocena = ROD::Matching(41, 41, alfa, Dimg, Mimg, Mono, aWzor2, AngleThresh_);


		    			if (ocena > ocenaDod)
		    				ocenaDod = ocena;
		    			Dimg.release();
		    			Mimg.release();
		    			Mono.release();
		    		}

		    	ocenaSzypulki.at<double>(i, 0) += ocenaDod;
		    	if (ocenaDod > ScoreThresh2)
		    	{
		    		// zachowaj krawędzie
		    		for(int m=x0-2;m<=x0+2;m++)
		    			for(int l=y0-2;l<=y0+2;l++)
		    				MaskaMonoSzyp.at<double>(l, m) = (double)EMImg_.at<uchar>(l,m);
		    	}
		    }
			
			
			// przesuwamy się cyklicznie w kółko z ograniczeniem DIRS - indeks właściwy to "i" 
			if (i ==  Wycinek_.at<int>(1,0) )
				break;
			i = i + 1;
		    if (i == DIRS) // przechodzimy przez -pi
		    	i = 0;
		}

		// Końcowa normalizacja
		double maxOcena;
		cv::Point maxInd;
		int xmin, xmax, ymin, ymax;

		cv::minMaxLoc(ocenaSzypulki,0,&maxOcena,0,&maxInd,cv::Mat());

		if (maxOcena > 0)
		{
		    ROD::jestSzypROD = 1;
		    ocenaSzypulki = 255.0 * ocenaSzypulki / maxOcena;

		    // ROI obszaru szypułki
		    x0 = pozycjaSzypulki_.at<int>(maxInd.y, 0);
		    y0 = pozycjaSzypulki_.at<int>(maxInd.y, 1);
		    xmin = x0 - 20;
			if (xmin < 0 ) xmin = 0;
		    xmax = x0 + 20;
			if (xmax >=sx) xmax = sx - 1;
		    ymin = y0 - 20;
			if (ymin<0) ymin = 0;
		    ymax = y0 + 20;
			if (ymax>=sy) ymax = sy-1;

		    roiSzypMono = (cv::Mat_<int>(4,1) << xmin, ymin, xmax, ymax);
		    katSzyp = ANGLES.at<double>(maxInd.x);//maxAngle
		    pozycjaSzypulki = (cv::Mat_<int>(2,1) <<x0, y0);
		}
		else
		{
		    ROD::jestSzypROD = 0;
			roiSzypMono = (cv::Mat_<int>(4,1) <<0,0,0,0);
			katSzyp = 0;
		    pozycjaSzypulki = (cv::Mat_<int>(2,1) <<0, 0);
		}
		
		ANGLES.release();
		Wzor1.release();
		Wzor2.release();
		aWzor1.release();
		aWzor2.release();
		pozycjaSzypulki_.release();
	}

