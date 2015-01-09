	
	//
	//
	// Funkcja pomocnicza 8: OgonekIR()
	//
	//jestOgonIr, pozIrOgon, alfaIr, CirxNew, CiryNew, MaskaIrOgon, BBoxIr;
	int jestOgonIr;
	::cv::Mat pozIrOgon = ::cv::Mat::zeros(2, 1, CV_32SC1); // wartoœci int
	::cv::Mat alfaIr = ::cv::Mat::zeros(4, 1, CV_64FC1); // wartosci double
	::cv::Mat MaskaIrOgon;// = ::cv::Mat::zeros(DIRS, 1, CV_32SC1);
	//::cv::Mat BBox = ::cv::Mat::zeros(4,1,CV_64FC1);

	cv::Mat OgonekIR(int my_, int mx_, double wspPIK_, int CxIn_, int CyIn_, cv::Mat &Wycinek_, 
		cv::Mat &Kont_, cv::Mat &Kont1D_, cv::Mat &Grad1D_,
		int &CirxN, int &CiryN)
	{
		// Parametr:
		// wspPIK_ = 120.0; //% przeskalowanie progu dla piku
		
		// Wewnêtrzny parametr:
		int zewnSKOK = 30; //% obszar zewnêtrzny

		// Jawnie zwracany wynik - nowy prostok¹t obejmuj¹cy jab³ko oraz promieñ
		::cv::Mat BBox_ = ::cv::Mat::zeros(5, 1, CV_32SC1); // wartoœci int

		// Przy braku ogonka zwróæ dane wejœciowe
		CirxN = CxIn_;
		CiryN = CyIn_;
		jestOgonIr = 0;
		 
		//
		double alfa1 = 0.0;
		double alfa2 = 0.0;
		double alfa0 = 0.0;
		double alfa3 = 0.0;

	//TEST
		/*
		cout<<"Kont1D_ :" ;
		for (int i=0; i<720; i++)
			cout << Kont1D_.at<double>(i,0) << "; " ;
		cout<< endl;
		cout<<"Grad1D_ :" ;
		for (int i=0; i<720; i++)
			cout << Grad1D_.at<double>(i,0) << "; " ;
		cout<< endl;
		*/
	// 


		//% Analiza funkcji 1D
		// Odchylenie standardowe
		cv::Mat stdValMat;
		//cv::Mat stdMeanMat;
		cv::meanStdDev(Grad1D_, cv::noArray(), stdValMat);//std(Grad1D(:));
		double stdVal=stdValMat.at<double>(0,0);
		stdValMat.release();

		//
		double gMax, gMin;
		int minInd, maxInd, minInd1, maxInd1, minInd2, maxInd2;
		cv::Point minIndP, maxIndP;
		double gMax1, gMin1, gMax2, gMin2;
		double maxVal, tStd;
		int llenBez, lOgon;
		int minX, minY, maxX, maxY, X, Y;
		int k, k0, ind0, ind1, indValue, ind2, kon;
		double value0, value, value2, value1;
		int A, B, A1, B1;


		// D³ugoœæ wektora danych
		int llen = Kont1D_.rows; // odpowiada aktulanej wartoœci DIRS
		int llenHalf = llen / 2; // llen musi byæ parzyste!

		MaskaIrOgon = ::cv::Mat::zeros(llen, 1, CV_8UC1);

		// ZnajdŸ maksimum i minimum wycinka lub ca³oœci 
		ind0 = Wycinek_.at<int>(0,0);
		ind1 = Wycinek_.at<int>(1,0);
		if (ind1 > ind0) // to jest normalna sytuacja
		{
			// Ewentualnie przytnij dane konturu
			::cv::Mat Grad1DPart = Grad1D_(::cv::Rect(0, ind0, 1, ind1 - ind0 + 1)).clone();
			// ZnajdŸ po³o¿enie min i max
		    cv::minMaxLoc(Grad1DPart.col(0), &gMin, &gMax, &minIndP, &maxIndP, cv::noArray());
		    maxInd = int(maxIndP.y); 
			minInd = int(minIndP.y);
		    maxInd = maxInd + ind0;
			minInd = minInd + ind0;
			Grad1DPart.release();
		}
		else // sprawdzany zakres przechodzi przez koniec/pocz¹tek
		{
			// oddzielnie badamy dwie czêœci
			// 1-sza czêœæ
			::cv::Mat Grad1D_ROI = Grad1D_(::cv::Rect(0, ind0, 1, llen - ind0)).clone();
			// po³o¿enie min i max w 1-szej czêœci
			cv::minMaxLoc(Grad1D_ROI.col(0), &gMin1, &gMax1, &minIndP, &maxIndP, cv::noArray());
			maxInd1 = int(maxIndP.y); 
			minInd1 = int(minIndP.y);
			maxInd1 = maxInd1 + ind0;
			minInd1 = minInd1 + ind0;
			
			Grad1D_ROI.release();
			// 2-ga czêœæ
			Grad1D_ROI = Grad1D_(::cv::Rect(0, 0, 1, ind1 + 1 )).clone();
			cv::minMaxLoc(Grad1D_ROI.col(0), &gMin2, &gMax2, &minIndP, &maxIndP, cv::noArray());
			maxInd2 = (int)maxIndP.y; 
			minInd2 = (int)minIndP.y;

			if (gMax1 > gMax2) {
				gMax = gMax1;
				maxInd = maxInd1;
			}
			else {
				gMax = gMax2;
				maxInd = maxInd2;
			}
			if (gMin1 < gMin2) {
				gMin = gMin1;
				minInd = minInd1;
			}
			else {
				gMin = gMin2;
				minInd = minInd2;
			}
			Grad1D_ROI.release();
		}

		// Maksymalna bezwglêdna wartoœæ
		maxVal = MAX(gMax, abs(gMin));
		
		// Pik gradientu musi posiadaæ znacz¹c¹ wartoœæ
		// adaptacyjny próg - zale¿y od aktualnego odchylenia standardowego
		tStd = wspPIK_ * stdVal; // = wspPIK (by³o oryg. 120, teraz 5.0 ?! ) * odchylenie standardowe

//TEST
		//std::cout<<"TEST OgonekIR: "<<maxVal<<" maxVal; "<<tStd<< " tStd; "<<"\n";
//~TEST
		// Sprawdzamy
		if (maxVal > tStd)
		{
			//% jest ogonek
			if (abs(gMin) > gMax)
			{
				// znajdŸ lokalne minimum w otoczeniu <-3, +6>
				k = minInd - 3;
				if (k < 0)
					k = k + llen;
				value0 = Kont1D_.at<double>(k,0); 
				for (int i=0; i < 9; i++) 
				{
					if (value0 > Kont1D_.at<double>(k,0))
						value0 = Kont1D_.at<double>(k,0); 
					k = k+1;
					if (k>=llen)
						k=0;
				}
				// ujemny pik - lokalne maksimum funkcji jest z ty³u
				MaskaIrOgon.at<uchar>(minInd,0) = 1;
				value = Kont1D_.at<double>(minInd,0); // szukamy maksimum odleg³oœci
				// Poprzedni element te¿ nale¿y i powinien byæ wiêkszy
				k = minInd - 1; // 
				if (k<0)
					k = llen - 1;
				MaskaIrOgon.at<uchar>(k,0) = 1;
				if (value < Kont1D_.at<double>(k,0))
					value = Kont1D_.at<double>(k,0); // tu spodziewamy siê maksimum
				
				// Nastêpny element ju¿ powinien nale¿eæ do konturu
				k = minInd + 1; 
				if (k>=llen)
					k = 0;
				MaskaIrOgon.at<uchar>(k,0) = 1;
				ind0 = k; // to jest na razie prawy koniec konturu szypu³ki
				
				// Przeszukujemy s¹siedztwo:
				// zaznaczamy fragment - o maksymalnie 10 elementach
				// dopóki wartoœæ odleg³oœci jest wiêksza od value0 i maleje wzglêdem pik-u 
				k = minInd - 2; 
				if (k < 0)
					k = llen - 1;
				indValue = k;
				for (int i= 0; i< 10; i++)
				{
					if (Kont1D_.at<double>(k,0) >= value0) // dopóki odlegloœæ jest wiêksza od poziomu konturu
					{
						value = Kont1D_.at<double>(k,0);
						indValue = k;
						MaskaIrOgon.at<uchar>(k,0) = 1;
					}
					else
						break;
					// ewentualnie przechodzi przez pocz¹tek
					k = k-1;
					if ( k < 0)
						k = llen-1;
				}
				// Indeks lewego koñca jest w ind1
				ind1 = indValue;
				value1 = value;
				MaskaIrOgon.at<uchar>(ind1, 0) = 1;
				
				// Przeszukujemy s¹siedztwo w prawo:
				// zaznaczamy fragment - o maksymalnie 10 elementach
				// dopóki wartoœæ odleg³oœci jest wiêksza od value0 
				k = minInd + 2; 
				if (k >= llen)
					k = k - llen;
				indValue = k;
				for (int i= 0; i< 10; i++)
				{
					if (Kont1D_.at<double>(k,0) >= value0) // dopóki odlegloœæ jest wiêksza od poziomu konturu
					{
						value = Kont1D_.at<double>(k,0);
						indValue = k;
						MaskaIrOgon.at<uchar>(k,0) = 1;
					}
					else
						break;
					// ewentualnie przechodzi przez pocz¹tek
					k = k+1;
					if ( k >= llen)
						k = 0;
				}
				// Indeks prawego koñca jest w ind0
				ind0 = indValue;
				value1 = value;
				MaskaIrOgon.at<uchar>(ind0, 0) = 1;

				// Teraz ponownie prawe zbocze piku ??? to jest b³êdne!
				// warunek na wczeœniejsze zakoñczenie
				//value2 = value0 + 0.2 *(value1 - value0);
				//k = ind1;
				//k0 = k - 50;
				//for (int i=ind1; i>= k0; i--)
				//{
				//	if (Kont1D_.at<double>(k,0) < value2)
				//		break;

				//	MaskaIrOgon.at<uchar>(k, 0) = 1;
				//	k = k-1;
				//	if (k<0)
				//		k = llen-1;
				//  }
				//ind2 = k;
				//MaskaIrOgon.at<uchar>(ind2, 0) = 1;
				
				// Ostatecznie ogonek jest indeksowany od lewego koñca (A) do prawego (B) 
				A = ind1;
				B = ind0;

				// Zewnêtrzna para wartoœci indeksów
				A1 = A - zewnSKOK;
				if (A1 < 0)
					A1 = llen + A1 - 1;
				B1 = B + zewnSKOK;
				if (B1 >= llen)
					B1 = B1 - llen;
			}
			else // dodatni pik
			{
				// lokalne maksimum funkcji jest z przodu
				// znajdŸ lokalne minimum w otoczeniu <-6, +3>
				k = maxInd - 6;
				if (k < 0)
					k = k + llen;
				value0 = Kont1D_.at<double>(k,0); 
				for (int i=0; i < 9; i++) 
				{
					if (value0 > Kont1D_.at<double>(k,0))
						value0 = Kont1D_.at<double>(k,0); 
					k = k+1;
					if (k>=llen)
						k=0;
				}
				//
				MaskaIrOgon.at<uchar>(maxInd,0) = 1;
				value = Kont1D_.at<double>(maxInd,0); // szukamy maksimum odleg³oœci
				// Nastêpny element te¿ nale¿y i powinien byæ wiêkszy
				k = maxInd + 1; // 
				if (k>=llen)
					k = 0;
				MaskaIrOgon.at<uchar>(k,0) = 1;
				if (value < Kont1D_.at<double>(k,0))
					value = Kont1D_.at<double>(k,0); // tu spodziewamy siê maksimum
				
				// Poprzedni element ju¿ powinien nale¿eæ do konturu
				k = maxInd - 1; 
				if (k <0)
					k = 0;
				MaskaIrOgon.at<uchar>(k,0) = 1;
				
				ind0 = k; // to jest na razie lewy koniec konturu szypu³ki

				// Przeszukujemy s¹siedztwo:
				// zaznaczamy fragment - o maksymalnie 10 elementach
				// dopóki wartoœæ odleg³oœci jest wiêksza od value0 i maleje wzglêdem pik-u 
				k = maxInd + 2; 
				if (k >= llen)
					k = k - llen;
				indValue = k;
				for (int i= 0; i< 10; i++) 
				{ 
					if (Kont1D_.at<double>(k,0) >= value0) // jest jeszcze wiêksza od poziomu konturu
					{
						value = Kont1D_.at<double>(k,0);
						indValue = k;
						MaskaIrOgon.at<uchar>(k, 0) = 1;
					}
					else
						break;

					k = k+1;
					if (k>=llen)
						k = 0;
				}
				// Prawy brzeg jest w ind1
				ind1 = indValue;
				value1 = value;
				MaskaIrOgon.at<uchar>(ind1, 0) = 1;
				
				// Przeszukujemy s¹siedztwo w lewo:
				// zaznaczamy fragment - o maksymalnie 10 elementach
				// dopóki wartoœæ odleg³oœci jest wiêksza od value0 
				k = maxInd - 2; 
				if (k < 0 )
					k = k + llen;
				indValue = k;
				for (int i= 0; i< 10; i++)
				{
					if (Kont1D_.at<double>(k,0) >= value0) // dopóki odlegloœæ jest wiêksza od poziomu konturu
					{
						value = Kont1D_.at<double>(k,0);
						indValue = k;
						MaskaIrOgon.at<uchar>(k,0) = 1;
					}
					else
						break;
					// ewentualnie przechodzi przez pocz¹tek
					k = k-1;
					if ( k < 0)
						k = llen-1;
				}
				// Indeks lewego koñca jest w ind0
				ind0 = indValue;
				value1 = value;
				MaskaIrOgon.at<uchar>(ind0, 0) = 1;

				//% prawe zbocze ?
				/*
				k = ind1;
				k0 = k+ 50;
				value2 = value0 + 0.2 *(value1 - value0);
				for (int i=ind1; i<= k0; i++)
				{
					if (Kont1D_.at<double>(k,0) < value2)
						break;
					MaskaIrOgon.at<uchar>(k, 0) = 1;
					k = k+1;
					if (k>=llen)
						k = 0;
				}
				ind2 = k;
				MaskaIrOgon.at<uchar>(ind2, 0) = 1;
				*/
				//% ostatecznie
				A = ind0;
				B = ind1;
				//% obejmuj¹ce wartoœci indeksów
				A1 = A - zewnSKOK;
				if (A1 < 0)
					A1 = llen + A1-1;

				B1 = B + zewnSKOK;
				if (B1 >= llen)
					B1 = B1 - llen;
			}
			

			// Oblicz nowy œrodek konturu bez ogonka
			// Oblicz bBox konturu bez ogonka
			lOgon = cv::sum(MaskaIrOgon).val[0]; // liczba punktów ogonka
			llenBez = llen - 2 * lOgon; // liczba punktów konturu bez ogonka
			minX = mx_;
			minY = my_;
			maxX = 0;
			maxY = 0;
			double sumX= 0.0;
			double sumY= 0.0;
			for (int i=0; i<llen; i++)
			{
				
				if (MaskaIrOgon.at<uchar>(i, 0) == 0)
				{
					X = (int) Kont_.at<double>(i,0);
					Y = (int) Kont_.at<double>(i,1);

					if (X > maxX)
						maxX = X;
					else if (X < minX)
						minX = X;
					if ( Y > maxY)
						maxY = Y;
					else if ( Y < minY)
						minY = Y;
					// czy dodaæ do wyznaczenia œrodka?
					k = i + llenHalf;
					if (k >= llen) 
						k = i - llenHalf;
					if (MaskaIrOgon.at<uchar>(k, 0) == 0) // tak, mo¿na uwzglêdniæ, po przek¹tnej nie ma ogonka
					{
						sumX = sumX + X; 
						sumY = sumY + Y;
					}
				}
			}

			// Nowy œrodek konturu jab³ka
			CirxN = round(sumX / llenBez);
			CiryN = round(sumY / llenBez);

			// oblicz k¹ty: wewnetrzne
			jestOgonIr = 1;
			alfa1 = atan2( -(Kont_.at<double>(A,1) - CiryN), Kont_.at<double>(A,0) - CirxN);
			alfa2 = atan2( -(Kont_.at<double>(B,1) - CiryN), Kont_.at<double>(B,0) - CirxN);
			// obejmuj¹ce - szersze
			alfa0 = atan2( -(Kont_.at<double>(A1,1) - CiryN), Kont_.at<double>(A1,0) - CirxN);
			alfa3 = atan2( -(Kont_.at<double>(B1,1) - CiryN), Kont_.at<double>(B1,0) - CirxN);
			
			// Wynik zwracany przez funkcjê
			// - k¹ty
			alfaIr.at<double>(0,0) = alfa1; 
			alfaIr.at<double>(1,0) = alfa2; 
			alfaIr.at<double>(2,0) = alfa3;
			alfaIr.at<double>(3,0) = alfa0;
			
			// - centrum szypu³ki (ogonka)
			
			X = round(0.5 * (Kont_.at<double>(A,0) + Kont_.at<double>(B,0)));
			Y = round(0.5 * (Kont_.at<double>(A,1) + Kont_.at<double>(B,1)));
			/* ???
			cv::Mat MeanPozIrOgon;
			MeanPozIrOgon.push_back(Kont_.at<double>(A,0));
			MeanPozIrOgon.push_back(Kont_.at<double>(B,0));
			X = cv::mean(MeanPozIrOgon,cv::Mat()).val[0];//[Kont_.at<double>(A,0), Kont_.at<double>(B,0)]).val[0];
			MeanPozIrOgon.pop_back(MeanPozIrOgon.rows);
			MeanPozIrOgon.push_back(Kont_.at<double>(A,1));
			MeanPozIrOgon.push_back(Kont_.at<double>(B,1));
			Y = cv::mean(MeanPozIrOgon,cv::Mat()).val[0];//[Kont_.at<double>(A,1), Kont_.at<double>(B,1)]).val[0];
			MeanPozIrOgon.pop_back(MeanPozIrOgon.rows);
			MeanPozIrOgon.release();
			*/
			pozIrOgon.at<int>(0,0) = X;
			pozIrOgon.at<int>(1,0) = Y;
			
		}
		else // brak widocznej szypu³ki odstaj¹cej
		{
			double minDouble, maxDouble;
			// brak widocznej szypu³ki (ogonka)
			lOgon = 0;
		    jestOgonIr = 0;
		    for(int l=0;l<=3;l++)
		    	alfaIr.at<double>(l,0) = 0;

		    pozIrOgon.at<int>(0,0) = -1;
		    pozIrOgon.at<int>(1,0) = -1;

		    cv::minMaxLoc(Kont_.col(0),&minDouble,&maxDouble,0,0,cv::Mat());
		    minX = round(minDouble); 
			maxX = round(maxDouble);
		    cv::minMaxLoc(Kont_.col(1),&minDouble,&maxDouble,0,0,cv::Mat());
		    minY = round(minDouble); 
			maxY = round(maxDouble);
		}

		//Oblicz promieñ konturu
		double sumD = 0.0;
		double dx, dy;
		
		llenBez = llen - 2 * lOgon;
		// llen musi byc parzyste!
		for (int i=0; i<llenHalf; i++)
		{
			if ((MaskaIrOgon.at<uchar>(i, 0) == 0)
				&& (MaskaIrOgon.at<uchar>(i, 0) == 0))
			{ 
				dx= Kont_.at<double>(i,0) - CirxN;
				dy= Kont_.at<double>(i,1) - CiryN;
				sumD += sqrt((double(dx*dx + dy*dy)));
				dx=Kont_.at<double>(i+llenHalf,0) - CirxN;
				dy=Kont_.at<double>(i+llenHalf,1) - CiryN;
				sumD += sqrt((double(dx*dx + dy*dy)));
			}
		}
		// zapisz promieñ
		BBox_.at<int>(4,0)=  round(sumD / llenBez);
		
		// Nowe ROI obszaru jab³ka
		BBox_.at<int>(0,0)= minX;
		BBox_.at<int>(1,0)= minY;
		BBox_.at<int>(2,0)= maxX;
		BBox_.at<int>(3,0)= maxY;

		return BBox_;
	}

