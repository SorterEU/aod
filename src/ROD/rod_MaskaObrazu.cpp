
	//
	// Funkcja pomocnicza 11: MaskaObrazuRGB()
	//

	//cv::Mat F1ColorImg;
	//cv::Mat F1MonoImg;
	//cv::Mat MaskImg;
	
	//double RGBTHRESH, MONOTHRESH;
	
	cv::Mat MaskaObrazuRGB(int NCy, int NCx,cv::Mat &NCImg_,cv::Mat  &NMImg_,cv::Mat &Kontur_)
	//, 		double vThrgb,double vThmono)
	{
		// Tworzenie maski obiektu w obrazach RGB i Mono
		// na podstawie wczeœniej znalezionego konturu
		// Wype³nianie wnêtrza konturu = 0
		//       brzeg konturu = 2
		//       zewnêtrze konturu = 1
		
		int sx, sy;
		cv::Mat MaskImg;
		//int d=3;
		std::vector<cv::Mat> NCImg_planes;
		int i,j;
		double x, y, x0, y0, x1, y1, x2, y2, dx, dy;
		double adx, ady, dist, ddx, ddy, curx, cury;

		int Cy = NCy;
		int Cx = NCx;
		sy=NCImg_.rows;
		sx=NCImg_.cols;
		
		MaskImg = cv::Mat::ones(sy,sx,CV_8UC1);//uchar(0,255)//CV_64FC1);

		//RGBTHRESH = vThrgb; //% przysz³e u¿ycie
		//MONOTHRESH = vThmono; //% przysz³e u¿ycie

		int ll = Kontur_.rows; // odpowiada liczbie DIRS
		int d = Kontur_.cols;
		
	// 1) Domkniêcie konturu - rozszerz kontur w razie potrzeby
		// Pierwszy punkt konturu
		x0 = Kontur_.at<double>(ll-1,0);
		y0 = Kontur_.at<double>(ll-1,1);
		MaskImg.at<uchar>(y0, x0) = 2; // 2 : brzeg
		for (i = 0; i<ll ; i++)
		{
			// Poprzedni punkt konturu
		    //x = Kontur_.at<double>(i-1,0);
		    //y = Kontur_.at<double>(i-1,1);
			x = x0;
			y = y0;
		    // bie¿¹cy punkt konturu
		    x0 = Kontur_.at<double>(i,0);
		    y0 = Kontur_.at<double>(i,1);
			MaskImg.at<uchar>(y0, x0) = 2; // 2 : brzeg
		    dx = x0 - x;
		    dy = y0 - y;
		    // czy odstêp?
		    adx = abs(dx);
		    ady = abs(dy);
		    if (adx > ady)
		        dist = adx;
		    else
		        dist = ady;
		    if (dist >= 1)
		    {
		        ddx = double(dx) / double(dist);
		        ddy = double(dy) / double(dist);
		        for (int d = 1; d<= dist; d++)
		        {
		            curx = round(x + d * ddx);
		            cury = round(y + d * ddy);
		            MaskImg.at<uchar>(cury, curx) = 2; //% 2= brzeg
					// wzmocnienie brzegu
					x1= curx - 1;
					if (x1<0)     x1 = 0;
					x2 = curx + 1;
					if (x2 >=sx)  x2 = sx-1;
					y1= cury - 1; 
					if (y1 < 0)   y1 = 0;
					y2 = cury + 1; 
					if (y2 >= sy) y2 = sy-1;
					MaskImg.at<uchar>(y1, x1) = 2; //s¹siedzi
					MaskImg.at<uchar>(y1, x2) = 2; //
					MaskImg.at<uchar>(y2, x1) = 2; //
					MaskImg.at<uchar>(y2, x2) = 2; //
					MaskImg.at<uchar>(y1, curx) = 2; //
					MaskImg.at<uchar>(y2, curx) = 2; //
					MaskImg.at<uchar>(cury, x1) = 2; //
					MaskImg.at<uchar>(cury, x2) = 2; //
		        }
		    }
		}

	// 2) Wype³niaj wnêtrze domkniêtego konturu
		int MAXL = sy * sx;
		cv::Mat pixList = cv::Mat::zeros(MAXL, 2, CV_64FC1);// % y, x
		cv::Mat OutImg_ = cv::Mat::zeros(sy, sx, CV_32SC1);

		int listInd = 0;
		int yc, xc;
		pixList.at<double>(listInd,0) = Cy;
		pixList.at<double>(listInd,1) = Cx;
		int nextInd = listInd + 1;

		// Z za³o¿enia œrodek musi le¿eæ wewn¹trz
		OutImg_.at<int>(Cy, Cx) = 1; // pierwszy piksel jest "zajêty"

//		F1ColorImg.at<uchar>(Cy, Cx) = NCImg_.at<uchar>(Cy, Cx);
//		F1MonoImg.at<uchar>(Cy, Cx) = NMImg_.at<uchar>(Cy, Cx);
		
		MaskImg.at<uchar>(Cy, Cx) = 0; // Œrodek masy to "seed point" dla wype³niania obszaru
		
		// Zamkniêcie obrysu - mo¿na do tego sprowadziæ drugi krok tej funkcji
		//cv::dilate(MaskImg,MaskImg,cv::noArray(),cv::Point(-1,-1),1,1,1);
		//cv::erode(MaskImg,MaskImg,cv::Mat(),cv::Point(-1,-1),1,1,1);
		//cv::morphologyEx(MaskImg,MaskImg,CV_MOP_CLOSE,cv::Mat(),cv::Point(),1,1,1);

		while (listInd < nextInd)
		{
		    yc = pixList.at<double>(listInd,0);
		    xc = pixList.at<double>(listInd,1);
		    //% 4 s¹siadów
		    //% 1
		    x1 = xc + 1;
		    y1 = yc;
		    if (x1 < sx)
		    {
		        if ((MaskImg.at<uchar>(y1,x1)!= 2) && (OutImg_.at<int>(y1,x1) == 0))// % jest wolny i nie jest brzegiem
		        {
		        	//% mo¿e byæ dodany do obszaru
		            pixList.at<double>(nextInd, 0) = y1;
		            pixList.at<double>(nextInd, 1) = x1;
		            OutImg_.at<int>(y1,x1) = 1; //% piksel staje siê "zajêty"
		            nextInd += 1;
		            //F1ColorImg.at<uchar>(y1, x1) = (NCImg_.at<uchar>(y1, x1));
		            //F1MonoImg.at<uchar>(y1, x1) = NMImg_.at<uchar>(y1, x1);
		            MaskImg.at<uchar>(y1, x1) = 0;
		        }
		    }
		    //%2
		    x1 = xc - 1;
		    y1 = yc;
		    if (x1 >= 0)
		        if ((MaskImg.at<uchar>(y1,x1)!= 2) && (OutImg_.at<int>(y1,x1) == 0) )//%
		        {
		            pixList.at<double>(nextInd, 0) = y1;
		            pixList.at<double>(nextInd, 1) = x1;
		            OutImg_.at<int>(y1,x1) = 1;// %
		            nextInd += 1;
		            //F1ColorImg.at<uchar>(y1, x1) = NCImg_.at<uchar>(y1, x1);
		            //F1MonoImg.at<uchar>(y1, x1) = NMImg_.at<uchar>(y1, x1);
		            MaskImg.at<uchar>(y1, x1) = 0;
		            
		        }
		    //%3
		    x1 = xc;
		    y1 = yc + 1;
		    if (y1 < sy)
		         if ( (MaskImg.at<uchar>(y1,x1)!= 2) && (OutImg_.at<int>(y1,x1) == 0) ) //%
		         {
		            pixList.at<double>(nextInd, 0) = y1;
		            pixList.at<double>(nextInd, 1) = x1;
		            OutImg_.at<int>(y1,x1) = 1; //%
		            nextInd +=1;
		            //F1ColorImg.at<uchar>(y1, x1) = NCImg_.at<uchar>(y1, x1);
		            //F1MonoImg.at<uchar>(y1, x1) = NMImg_.at<uchar>(y1, x1);
		            MaskImg.at<uchar>(y1, x1) = 0;
		            
		         }
		    //%4
		    x1 = xc;
		    y1 = yc - 1;
		    if (y1 >= 0)
		         if ( (MaskImg.at<uchar>(y1,x1)!= 2) && (OutImg_.at<int>(y1,x1) == 0) )//%
		         {
		            pixList.at<double>(nextInd, 0) = y1;
		            pixList.at<double>(nextInd, 1) = x1;
		            OutImg_.at<int>(y1,x1) = 1; //%
		            nextInd += 1;
		            //F1ColorImg.at<uchar>(y1, x1) = NCImg_.at<uchar>(y1, x1);
		            //F1MonoImg.at<uchar>(y1, x1) = NMImg_.at<uchar>(y1, x1);
		            MaskImg.at<uchar>(y1, x1) = 0;
		            
		         }
		    listInd = listInd + 1;
		}

	// 3) Maj¹c maskê - maskuj wejœciowe obrazy RGB i mono
		// Uwaga: obrazy te zostanê zmodyfikowane!
		cv::Mat NCImg_temp = NCImg_;
		cv::split(NCImg_temp,NCImg_planes); // dostêp do 3 p³atów obrazu RGB
		
		for(int i=0; i<MaskImg.rows; i++)
			for(int j=0; j<MaskImg.cols; j++)
				if(MaskImg.at<uchar>(i, j)!=0)
				{
					for(int k = 0; k < 3; k++) 
						NCImg_planes[k].at<uchar>(i,j) = 255;
					NMImg_.at<uchar>(i, j)=255;
				}

		cv::merge(NCImg_planes,NCImg_temp); // Ponownie po³¹cz p³aty w obraz

		pixList.release();
		OutImg_.release();
		NCImg_temp.release();
		
		// Zwraca jawnie maskê obrazu RGB/Mono
		return MaskImg;
	}


	//
	//
	// Funkcja pomocnicza 12: MaskaObrazuIR()
	//
	//cv::Mat F1IRImg;
	//cv::Mat IRMaskImg;

	cv::Mat MaskaObrazuIR(int NCy_,int NCx_,cv::Mat &NGImg_,cv::Mat &Kontur_)//,double vThresh_)
	{
		//% Tworzenie maski obiektu w obrazie IR
		//% na podstawie wczeœniej znalezionego konturu
		//% Wype³nianie wnêtrza konturu

		int Cirx = NCx_;
		int Ciry = NCy_;
		int sy, sx;
		sy = NGImg_.rows;
		sx = NGImg_.cols;
		double x,y,x0,y0,dx,dy,adx,ady,dist;
		double ddx, ddy;
		int curx, cury, x1,x2,y1,y2,xc,yc;
		cv::Mat F1GImg = (255 * cv::Mat::ones(sy, sx,CV_8UC1));

		cv::Mat IRMaskImg_ = cv::Mat::ones(sy, sx, CV_8UC1);

		//double IRTHRESH = vThresh_;

		int ll = Kontur_.rows;

	//1) Domknij kontur
		// Pierwszy punkt konturu
		x0 = Kontur_.at<double>(ll-1,0);
		y0 = Kontur_.at<double>(ll-1,1);
		IRMaskImg_.at<uchar>(y0, x0) = 2; // 2 : brzeg
		for (int i = 0; i< ll;i++)
		{
		    //% Poprzedni punkt konturu
		    //x = Kontur_.at<double>(i-1,0);
		    //y = Kontur_.at<double>(i-1,1);
			x = x0;
			y = y0;
		    //% bie¿¹cy punkt konturu
		    x0 = Kontur_.at<double>(i,0);
		    y0 = Kontur_.at<double>(i,1);
		    dx = x0 -x;
		    dy = y0 - y;
		    //% czy odstêp?
		    adx = abs(dx);
		    ady = abs(dy);
		    if (adx > ady)
		        dist = adx;
		    else
		        dist = ady;

		    if (dist >= 1)
		    {
		        ddx = double(dx) / double(dist);
		        ddy = double(dy) / double(dist);
		        for (int d = 1; d<=dist; d++)
		        {
		            curx = round(x + d * ddx);
		            cury = round(y + d * ddy);
		            IRMaskImg_.at<uchar>(cury, curx) = 2;// % 2= brzeg
					// wzmocnienie brzegu
					x1= curx - 1;
					if (x1<0)     x1 = 0;
					x2 = curx + 1;
					if (x2 >=sx)  x2 = sx-1;
					y1= cury - 1; 
					if (y1 < 0)   y1 = 0;
					y2 = cury + 1; 
					if (y2 >= sy) y2 = sy-1;
					IRMaskImg_.at<uchar>(y1, x1) = 2; //s¹siedzi
					IRMaskImg_.at<uchar>(y1, x2) = 2; //
					IRMaskImg_.at<uchar>(y2, x1) = 2; //
					IRMaskImg_.at<uchar>(y2, x2) = 2; //
					IRMaskImg_.at<uchar>(y1, curx) = 2; //
					IRMaskImg_.at<uchar>(y2, curx) = 2; //
					IRMaskImg_.at<uchar>(cury, x1) = 2; //
					IRMaskImg_.at<uchar>(cury, x2) = 2; //
		        }
		    }

		    // x1= x0 - 1;
		    // if (x1<0) 	x1 = 0;
		    // x2 = x0 + 1;
		    // if (x2 >=sx)  x2 = sx-1;
		    // y1= y0 - 1;
		    // if (y1 < 0)  y1 = 0;
		    // y2 = y0 + 1;
		    // if (y2 >=sy)  y2 = sy-1;
		    // for(int m=y1; m<=y2; m++)
		    	// for(int n=x1; n<=x2; n++)
		    		//IRMaskImg_.at<uchar>(m, n) = 2; //% 2= brzeg
		
		}
		
	//2) Wype³nij wnêtrze

		int MAXL = sy * sx;
		cv::Mat pixList = cv::Mat::zeros(MAXL, 2, CV_32SC1); //% y, x
		cv::Mat OutImg = cv::Mat::zeros(sy, sx, CV_32SC1);
		int listInd = 1;
		int nextInd;

		pixList.at<int>(listInd,0) = Ciry;
		pixList.at<int>(listInd,1) = Cirx;
		nextInd = listInd + 1;
		OutImg.at<int>(Ciry, Cirx) = 1; //% pierwszy piksel jest "zajêty"

		F1GImg.at<uchar>(Ciry, Cirx) = NGImg_.at<uchar>(Ciry, Cirx);
		IRMaskImg_.at<uchar>(Ciry, Cirx) = 0;

		while (listInd < nextInd)
		{
			yc = pixList.at<int>(listInd,0);
			xc = pixList.at<int>(listInd,1);
			//% 4 s¹siadów
			//% 1
			x1 = xc + 1;
			y1 = yc;
			if (x1 <= sx)
				if ((IRMaskImg_.at<uchar>(y1,x1)!= 2) && (OutImg.at<int>(y1,x1) == 0) )//% jest wolny i nie jest brzegiem
				{
					//% mo¿e byc dodany do obszaru
					pixList.at<int>(nextInd, 0) = y1;
					pixList.at<int>(nextInd, 1) = x1;
					OutImg.at<int>(y1,x1) = 1; //% piksel staje siê "zajêty"
					nextInd = nextInd + 1;
					F1GImg.at<uchar>(y1, x1) = NGImg_.at<uchar>(y1, x1);
					IRMaskImg_.at<uchar>(y1, x1) = 0;
				}

			//%2
			x1 = xc - 1;
			y1 = yc;
			if (x1 > 0)
				if ((IRMaskImg_.at<uchar>(y1,x1)!= 2) && (OutImg.at<int>(y1,x1) == 0) )// %
				{
					pixList.at<int>(nextInd, 0) = y1;
					pixList.at<int>(nextInd, 1) = x1;
					OutImg.at<int>(y1,x1) = 1; //%
					nextInd = nextInd + 1;
					F1GImg.at<uchar>(y1, x1) = NGImg_.at<uchar>(y1, x1);
					IRMaskImg_.at<uchar>(y1, x1) = 0;
				}

			//%3
			x1 = xc;
			y1 = yc + 1;
			if (y1 <= sy)
				if ((IRMaskImg_.at<uchar>(y1,x1)!= 2) && (OutImg.at<int>(y1,x1) == 0) )//%
				{
					pixList.at<int>(nextInd, 0) = y1;
					pixList.at<int>(nextInd, 1) = x1;
					OutImg.at<int>(y1,x1) = 1; //%
					nextInd = nextInd + 1;
					F1GImg.at<uchar>(y1, x1) = NGImg_.at<uchar>(y1, x1);
					IRMaskImg_.at<uchar>(y1, x1) = 0;
				}

			//%4
			x1 = xc;
			y1 = yc - 1;
			if (y1 > 0)
				if ((IRMaskImg_.at<uchar>(y1,x1)!= 2) && (OutImg.at<int>(y1,x1) == 0) )//%
				{
					pixList.at<int>(nextInd, 0) = y1;
					pixList.at<int>(nextInd, 1) = x1;
					OutImg.at<int>(y1,x1) = 1; //%
					nextInd = nextInd + 1;
					F1GImg.at<uchar>(y1, x1) = NGImg_.at<uchar>(y1, x1);
					IRMaskImg_.at<uchar>(y1, x1) = 0;
				}

			listInd = listInd + 1;
		// cout<< listInd << ", nextInd: " << nextInd << endl;

		}
	
		// Obraz wejœciowy IR jest teraz zmodyfikowany mask¹
		// Niejawny wynik zwracany przez parametr
		NGImg_ = F1GImg.clone();

		// Zwraca te¿ jawnie maskê
		return IRMaskImg_;
	}
