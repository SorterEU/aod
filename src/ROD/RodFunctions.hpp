// Aplikacja CAppSorter
// "Detekcja uszkodze� powierzchni jab�ka
// Funkcje pomocnicze dla klasy Analysis
// Autor: W�odzimierz Kasprzak
// Kod w C++: Jan Figat i W�odzimierz Kasprzak
// Ostatnia modyfikacja: 18-12-2014 (W. Kasprzak)
// Obszary nazw:
// - ROD : receptor agenta OD - do detekcji i oceny defektów powierzchni jabłka - 
//		- - klasa główna to "Analysis" - definicja w pliku Analysis.h
// - SFS : analiza 3D (Autorka: Karolina Przerwa)- klasa główna to "CALIBRATOR" 
//	    - - opis poarametrów w pliku Calibrator.HPP
// - AW : analiza 2D obrazu IR i Mono (uzyskanego z RGB) (Autor: Artur Wilkowski)


#ifndef RODFUNCTIONS
#define RODFUNCTIONS

#include <opencv/cv.h>
//#include <cv.h>

#include <opencv2/core/core.hpp>

using namespace std;


/// W zasadzie makro - potrzebne do zaokraglenia liczby double do int
/*int round(double liczba)
{
    return (liczba >= 0) ? (int)(liczba + 0.5) : (int)(liczba - 0.5);
}*/

// Lista funkcji:
// 1) EdgeDetection
// 2) ErosionEllipse
// 3) DilationEllipse
// 4) DetectROI
// 5) DetectROI2
// 6) KonturRGB
// 7) KonturIR
// 8) OgonekIR
// 9) GenAktWzorzec
// 10) SzypulkaMono
// 11) MaskaObrazuRGB
// 12) MaskaObrazuIR
// 13) RGBtoYbr

// Funkcje są w osobnych plikach - będą dołączane do tego pliku!

namespace ROD {
	
#include "rod_EdgeDetection.cpp"

	//
	//

#include "rod_Ellipse.cpp"	

	
#include "rod_DetectROI.cpp"	

	
#include "rod_DetectROI2.cpp"	
	

#include "rod_KonturRGB.cpp"


#include "rod_KonturIR.cpp"



#include "rod_OgonekIR.cpp"

#include "rod_SzypulkaMono.cpp"


#include "rod_MaskaObrazu.cpp"


	

	//
	//
	// Funkcja pomocnicza 13: RGBtoYbr()
	//

	//int maxYValue;
	cv::Mat YbrImg;
	cv::Mat RGBtoYbr(int my_,int mx_,int BGswitch, 
		int* maxYValue, cv::Mat &SObraz_,cv::Mat &Mask_)
	{
		// Przeksztalca obraz RGB w YCgCr lub YCrCb:
		// - rozmiar obrazu (my,mx)
		// - BGSwitch - tryb koloru wyjściowego obrazu. BGSwitch: 0= YCgCr, 1 = YCbCr
		//
		cv::Mat YbrImg_;
		//std::vector<cv::Mat> SObraz_planes(3);
		std::vector<cv::Mat> YbrImg_planes(3);

	// Uwaga: zamiana z BGR na RGB !

		//YbrImg_ = SObraz_.clone(); //BGR in openCV // RGB in Matlab
		cv::cvtColor(SObraz_,YbrImg_,CV_BGR2RGB); //jeśli potrzebna jest zamiana na RGB
		
		//cv::split(SObraz_,SObraz_planes);
		cv::split(YbrImg_,YbrImg_planes);
		cv::Mat A;
		cv::Mat vec;
		cv::Mat result;

		if (BGswitch == 0)
		{

		    // zamień na Y Cg Cr
		    A = (cv::Mat_<double>(3,3) <<0.299, 0.587, 0.114,
		     -0.3620, 0.5, -0.1380,  // wrt Green
		    0.5, -0.4187, -0.0813); // wrt Red
		}
		else
		{
		    // zamień na Y Cb Cr
		     A = (cv::Mat_<double>(3,3) <<0.299, 0.587, 0.114,
		     -0.1687, -0.3313, 0.5, // wrt Blue
		    0.5, -0.4187, -0.0813); // wrt Red
		}

		*maxYValue = 0; // maksymalna wartość jasności dla pikseli obszaru ROI
		
		int pix;
		for (int i=0;i<my_;i++)
		   for (int j=0;j<mx_;j++)
		   {
		       if (Mask_.at<uchar>(i,j) == 0)//  piksel należy do obszaru obiektu 
		       {
		    	   //
		           vec = (cv::Mat_<double>(3,1) << YbrImg_planes[0].at<uchar>(i,j), YbrImg_planes[1].at<uchar>(i,j), YbrImg_planes[2].at<uchar>(i,j));//
		           result = A * vec;
				   
				   pix = (round)(result.at<double>(0,0));
				   if (pix<0) pix = 0;
				   if (pix>255) pix = 255;
				   YbrImg_planes[0].at<uchar>(i, j) = (uchar)pix; //
				   
				   pix = 128 + (round)(result.at<double>(1,0));
				   if (pix<0) pix = 0;
				   if (pix>255) pix = 255;
				   YbrImg_planes[1].at<uchar>(i, j) = (uchar)pix;
				   
				   pix = 128 + (round)(result.at<double>(2,0));
				   if (pix<0) pix = 0;
				   if (pix>255) pix = 255;
				   YbrImg_planes[2].at<uchar>(i, j) = (uchar)pix; //

				   if (*maxYValue < result.at<double>(0,0))
		               *maxYValue = result.at<double>(0,0);

		           vec.release();
		           result.release();
		       }
		       else
		       {

		    	   YbrImg_planes[0].at<uchar>(i,j) = 255;
		    	   YbrImg_planes[1].at<uchar>(i,j) = 255;
		    	   YbrImg_planes[2].at<uchar>(i,j) = 255;
		       }
		   }

		cv::merge(YbrImg_planes,YbrImg_);
		
		A.release();
		return YbrImg_;
	}

	//
	//
	// Funkcja pomocnicza 14: NormYBR()
	//

	cv::Mat NormYBR(int my,int  mx,double KB,double KR,int maxYval,double normY,cv::Mat &SObraz,cv::Mat &Mask)
	{
		//%
		// Normalizuje obraz YCbCr względem jasności
		// - rozmiar obrazu (my,mx)
		// - kb, kr : współczynniki normalizacji dla składowych Cb i Cr

		
		double NORMY = normY; // % 66 dla typ 1
		//% 120 dla typ 2 i 3?
		//% Ewentualnie zależne od zakresu Y
		//%NORMY = (maxYval + 1.0) / 2.0;

		cv::Mat OutImg = SObraz.clone();
		cv::Mat vecPix;
		double delY;
		std::vector<cv::Mat> OutImg_planes;
		::cv::split(OutImg,OutImg_planes);

		for (int i=0;i<my;i++)
		   for (int j=0;j<mx;j++)
		       if (Mask.at<uchar>(i,j) == 0)// % is not a masked pixel
		       {
		           vecPix = (cv::Mat_<double>(3,1) <<OutImg_planes[0].at<uchar>(i,j), OutImg_planes[1].at<uchar>(i,j), OutImg_planes[2].at<uchar>(i,j));
		           delY = NORMY - vecPix.at<double>(0,0);
		           //OutImg_planes[0].at<uchar>(i,j) = vecPix.at<double>(0,0);
		           OutImg_planes[1].at<uchar>(i,j) = vecPix.at<double>(1,0) + delY * KB ;
		           OutImg_planes[2].at<uchar>(i,j) = vecPix.at<double>(2,0) + delY * KR ;
		       }
		return OutImg;
	}


	//
	//
	// Funkcja pomocnicza 15: MaskedVarianceHisto()
	//

	cv::Mat StdImgG, StdImgR;
	double resStdDevCBY,resStdDevCBX;
	
	cv::Mat MaskedVarianceHisto(int my,int mx,cv::Mat &SObraz,cv::Mat &Mask)
	{
		//% Compute three std. deviation histograms of the masked (Cb, Cr) image
		int MASK_SIZE = 3;
		int FINAL = MASK_SIZE - 1;
		int TwoFINAL = FINAL + MASK_SIZE; //% the borders are cut
		int STEP = 2;
		int SCALE = 80;

		//% The histogram of variance levels
		cv::Mat hist_ = cv::Mat::zeros(256,2,CV_32FC1);//float
		
		//% How many potential blocks exist
		StdImgG = Mask.clone();
		StdImgR = StdImgG.clone();
		
		int rowInd = -1;//
		int BlNum = 0;
		cv::Mat numColumns = cv::Mat::zeros(1,256,CV_8UC1);
		int numBlocks = 0;
	    double averBlocks = 0;
	    int columnInd;
	    int IND;
	    cv::Mat block;

	    std::vector<cv::Mat> SObraz_planes;
	    cv::split(SObraz,SObraz_planes);
	    
		cv::Mat stdValMat;
	    //cv::Mat stdMeanMat;
	    double stdVal;
	    int stdValInt;
	    cv::Mat MatROI;


	    for (int i=8-1;i<my-TwoFINAL;i+= STEP) //%
		{
		    numBlocks = 0;
		    averBlocks = 0.0;
		    columnInd = -1;//
		    rowInd = rowInd + 1;
		    for (int j=8-1;j<mx-TwoFINAL;j+=STEP) //%
		    {
		       columnInd = columnInd + 1;
		         //%
		       if ((Mask.at<uchar>(i,j) == 0) && (Mask.at<uchar>(i+FINAL,j+FINAL) == 0) && (Mask.at<uchar>(i, j+FINAL) == 0) && (Mask.at<uchar>(i+FINAL, j) == 0))
		       {
		    	   //% To jest piksel wewnątrz konturu
		           //%First feature: the histogramn of Cb (CG) variances
		           IND = 0;
		           block = SObraz_planes[1](cv::Rect(j,i,FINAL+1,FINAL+1));
		           cv::meanStdDev(block, /*stdMeanMat*/cv::noArray(), stdValMat);//%Uses the Cb (CG) plane
		           stdVal = stdValMat.at<double>(0,0);
		           //stdMeanMat.release();
		           stdValMat.release();
		           //%standard deviation within a block of MASK_SIXZE x MASK_SIZE
		           stdVal = SCALE * stdVal; //% scale it for presentation
		           stdValInt = round(stdVal);
		           if (stdValInt > 255) stdValInt = 255;//256
		           if (stdValInt < 0) stdValInt = 0;//1
		           hist_.at<float>(stdValInt,IND) += 1;
				   
		           //% set the image info
		           //StdImgG_ROI = StdImgG(cv::Range(i,i+FINAL+1),cv::Range(j,j+FINAL+1));
		           for (int m=i;m<=i+FINAL;m++)
		        	   for (int l=j;l<=j+FINAL;l++)
		        		   StdImgG.at<uchar>(m,l)=stdValInt;//-1
				
		           //%Second feature: the histogramn of Cr variances
		           IND = 1;
		           block.release();
		           //%valMean = mean(mean(double(SObraz(i: i+7,j: j+7,IND))));
		           block = SObraz_planes[2](cv::Rect(j,i,FINAL+1,FINAL+1));
		           cv::meanStdDev(block, /*stdMeanMat*/cv::noArray(), stdValMat);
		           stdVal = stdValMat.at<double>(0,0);
		           stdVal = SCALE * stdVal;
		           stdValInt = round(stdVal);
		           if (stdValInt > 255) stdValInt = 255;//256
		           if (stdValInt < 0) stdValInt = 0;//1
		           hist_.at<float>(stdValInt,IND) += 1;
		           //% set the image info
		           for (int m=i;m<=i+FINAL;m++)
		        	   for (int l=j;l<=j+FINAL;l++)
		        		   StdImgG.at<uchar>(m,l)=stdValInt;//-1

		           //% Third feature: the areal distribution of the mean value in Cb (seems to be larger then in Cr)
		           //IND = 0;
		           //%mVal = mean(mean(double(SObraz_planes[2](cv::Rect(j,i,FINAL+1,FINAL+1)))));
		           //%averBlocks += mVal;
		           numBlocks = numBlocks + 1; //% number of relevant blocks in current row

		           //% Fourth feature: the areal distribution of the mean value in Cb (seems to be larger then in Cr)
		           //IND = 1;
		           //hist_.at<float>(columnInd, 1) += mVal;
		           //numColumns.at<uchar>(columnInd) += 1; //% number of relevant blocks in current column
		           //%input('?');
		           BlNum = BlNum + 1;
		       }
		    }
		   //%if (numBlocks > 1)
		   //%    averBlocks = averBlocks / numBlocks;
		}
		//%hist(rowInd,1) = averBlocks;


		for (int k=rowInd; k<256;k++)
			hist_.at<float>(k, 0) = 128; //% for better presentation - we set the remaining values to a center value by default
		for (int k=0; k<=columnInd;k++)
		{
			if (numColumns.at<uchar>(k) > 2)
				hist_.at<float>(k, 1) = hist_.at<float>(k,1) / float(numColumns.at<uchar>(k)); //% compute the average per column value
			if (numColumns.at<uchar>(k) == 0) //% no block in given column
				hist_.at<float>(k,1) = 128; //% set a default value
		}

		for (int k=columnInd+1;k<256;k++)
			hist_.at<float>(k,1) = 128; //% for better presentation - we set the remaining values to a center value by default

		//%establish the std deviations of aerial histograms:
		int minIndYHist = 4; //%
		int maxIndYHist = rowInd - 5;
		int minIndXHist = 5;
		int maxIndXHist = columnInd - 5;

		MatROI= hist_(cv::Rect(0,minIndYHist,1,maxIndYHist-minIndYHist+1));
		cv::meanStdDev(MatROI, cv::noArray(), stdValMat);
		resStdDevCBY=stdValMat.at<double>(0,0); MatROI.release();
		stdValMat.release();
		MatROI = hist_(cv::Rect(1,minIndXHist,1,maxIndXHist-minIndXHist+1));
		cv::meanStdDev(MatROI,cv::noArray(), stdValMat);
		resStdDevCBX=stdValMat.at<double>(0,0);MatROI.release();
		stdValMat.release();

		//% Histogramy 3 i 4 nie sa na razie potrzebne

		//return
	    numColumns.release();
	    block.release();
		return hist_;
	}


	//
	//
	// Funkcja pomocnicza 16: Regions()
	//

	int regNum;
	cv::Mat SegmentImg, regionDesc;
	
	cv::Mat Regions(int my,int mx,cv::Mat &GImg,cv::Mat &RImg,cv::Mat &Mask,cv::Mat &StdR)
	{
		int MAXVAL = 4; //%homogenenity threshold
		int MAXSTD = 220; //% std val threshold
		cv::Mat OutImg = cv::Mat::zeros(my, mx,CV_32SC1);

		int MAXL = my * mx;
		cv::Mat pixList = cv::Mat::zeros(MAXL, 2, CV_32SC1); //% y, x //uchar
		int MAXR = 10 * my;
		cv::Mat regions = cv::Mat::zeros(MAXR, 6,CV_64FC1);//+ 0.00001; //% g val, r val, size, cy, cx, stdR
		cv::Mat regionsTemp;
		regNum = -1;
		int listInd, nextInd;
		double G,R,S;
		int stdregion, yc, xc, x1, y1;// x2, y2;
		double Ga, delG, Ra, delR, Sa;
		cv::Mat ROI;

		for (int y=0;y<my;y++)
			for (int x=0;x<mx;x++)
			{
				if ((Mask.at<uchar>(y,x) == 0) && (OutImg.at<int>(y,x) == 0))
				{
					//std::cout << "TEST -:) : regNum=" << regNum<<"\n";
					//%next start piksel
					regNum += 1;
					if (regNum >= MAXR) //% extend the region list
					{
						//std::cout<<regNum<<"___\n";
						regionsTemp = regions;
						regions = cv::Mat::zeros(2 * MAXR, 6,CV_64FC1);
						for(int i=0;i<MAXR;i++)
							for(int j=0;j<6;j++)
								regions.at<double>(i,j)=regionsTemp.at<double>(i,j);
						MAXR = 2* MAXR;
					}
					listInd = 0;//1;
					
					G = double(GImg.at<uchar>(y,x));
					R = double(RImg.at<uchar>(y,x));
					S = double(StdR.at<uchar>(y,x));
					regions.at<double>(regNum,0) = G;
					regions.at<double>(regNum,1) = R;
					regions.at<double>(regNum,2) = 1; //%size
					regions.at<double>(regNum, 5) = S;
					
					if (S > MAXSTD)
						stdregion = 1;
					else
						stdregion = 0;

					pixList.at<int>(listInd,0) = y;
					pixList.at<int>(listInd,1) = x;
					nextInd = listInd + 1;
					OutImg.at<int>(y,x) = regNum; //% pixel is "busy"
					
					while (listInd < nextInd  && MAXL>nextInd)
					{
						//std::cout << "nextInd" << nextInd << "\n";
						//% get pixel
						yc = pixList.at<int>(listInd,0);
						xc = pixList.at<int>(listInd,1);
						//% find 4 neighbours
						//% 1
						x1 = xc + 1;
						y1 = yc;
						if (x1 < mx)
						{
							if ((Mask.at<uchar>(y1,x1) == 0) && (OutImg.at<int>(y1,x1) == 0)) //% is free
							{
								//%check the homogeneity
								Ga = double(GImg.at<uchar>(y1,x1));
								delG = Ga - G;
								Ra = double(RImg.at<uchar>(y1,x1));
								delR = R - Ra;
								Sa = double(StdR.at<uchar>(y1,x1));
								if ( ((stdregion == 0) && (abs(delR) < MAXVAL) && (abs(delG) <= MAXVAL)) || ((stdregion == 1) && (Sa > MAXSTD)) )
								{
									//%can be added
									//% nowe wartości średnie dla obszaru
									G = G * regions.at<double>(regNum, 2) + Ga;
									R = R * regions.at<double>(regNum, 2) + Ra;
									S = S * regions.at<double>(regNum, 2) + Sa;
									regions.at<double>(regNum,2) += 1;
									G = G / regions.at<double>(regNum, 2);
									R = R / regions.at<double>(regNum, 2);
									S = S / regions.at<double>(regNum, 2);

									regions.at<double>(regNum,0) = G;
									regions.at<double>(regNum,1) = R;
									regions.at<double>(regNum,5) = S;
									if (nextInd< MAXL){
										pixList.at<int>(nextInd, 0) = y1;
										pixList.at<int>(nextInd, 1) = x1;
									}
									//else std::cout << "ERROR pixList.rows <=" << nextInd << " \n";
									OutImg.at<int>(y1,x1) = regNum; //% piksel "jest zajęty"
									nextInd = nextInd + 1;
								}
							}
						}
						//%2
						x1 = xc - 1;
						y1 = yc;
						if (x1 >= 0)
						{
							if ((Mask.at<uchar>(y1, x1) == 0) && (OutImg.at<int>(y1, x1) == 0)) //% is free
							{
								//%check the homogeneity
								Ga = double(GImg.at<uchar>(y1, x1));
								delG = Ga - G;
								Ra = double(RImg.at<uchar>(y1, x1));
								delR = R - Ra;
								Sa = double(StdR.at<uchar>(y1, x1));
								if (((stdregion == 0) && (abs(delR) < MAXVAL) && (abs(delG) <= MAXVAL)) || ((stdregion == 1) && (Sa > MAXSTD)))
								{
									//%can be added
									//% nowe wartości średnie dla obszaru
									G = G * regions.at<double>(regNum, 2) + Ga;
									R = R * regions.at<double>(regNum, 2) + Ra;
									S = S * regions.at<double>(regNum, 2) + Sa;
									regions.at<double>(regNum, 2) += 1;
									G = G / regions.at<double>(regNum, 2);
									R = R / regions.at<double>(regNum, 2);
									S = S / regions.at<double>(regNum, 2);

									regions.at<double>(regNum, 0) = G;
									regions.at<double>(regNum, 1) = R;
									regions.at<double>(regNum, 5) = S;
									if (nextInd < MAXL){
										pixList.at<int>(nextInd, 0) = y1;
										pixList.at<int>(nextInd, 1) = x1;
									}
									//else std::cout << "ERROR pixList.rows <=" << nextInd<<" \n";
									OutImg.at<int>(y1, x1) = regNum; //% piksel "jest zajęty"
									nextInd = nextInd + 1;
								}
							}
						}
						//%3
						x1 = xc;
						y1 = yc + 1;
						if (y1 < my)
						{
							if ((Mask.at<uchar>(y1,x1) == 0) && (OutImg.at<int>(y1,x1) == 0)) //% is free
							{
								//%check the homogeneity
								Ga = double(GImg.at<uchar>(y1,x1));
								delG = Ga - G;
								Ra = double(RImg.at<uchar>(y1,x1));
								delR = R - Ra;
								Sa = double(StdR.at<uchar>(y1,x1));
								if ( ((stdregion == 0) && (abs(delR) < MAXVAL) && (abs(delG) <= MAXVAL)) || ((stdregion == 1) && (Sa > MAXSTD)) )
								{
									//%can be added
									//% nowe wartości średnie dla obszaru
									G = G * regions.at<double>(regNum, 2) + Ga;
									R = R * regions.at<double>(regNum, 2) + Ra;
									S = S * regions.at<double>(regNum, 2) + Sa;
									regions.at<double>(regNum,2) += 1;
									G = G / regions.at<double>(regNum, 2);
									R = R / regions.at<double>(regNum, 2);
									S = S / regions.at<double>(regNum, 2);

									regions.at<double>(regNum,0) = G;
									regions.at<double>(regNum,1) = R;
									regions.at<double>(regNum, 5) = S;
									if (nextInd < MAXL){
										pixList.at<int>(nextInd, 0) = y1;
										pixList.at<int>(nextInd, 1) = x1;
									}
									OutImg.at<int>(y1,x1) = regNum; //% pixel is "busy"
									nextInd = nextInd + 1;
								}
							}
						}
						//%4
						x1 = xc;
						y1 = yc - 1;
						if (y1 >= 0)
						{
							if ((Mask.at<uchar>(y1,x1) == 0) && (OutImg.at<int>(y1,x1) == 0)) //% is free
							{
								//%check the homogeneity
								Ga = double(GImg.at<uchar>(y1,x1));
								delG = Ga - G;
								Ra = double(RImg.at<uchar>(y1,x1));
								delR = R - Ra;
								Sa = double(StdR.at<uchar>(y1,x1));
								if (((stdregion == 0) && (abs(delR) < MAXVAL) && (abs(delG) <= MAXVAL)) || ((stdregion == 1) && (Sa > MAXSTD)) )
								{
									//%can be added
									//% nowe wartości średnie dla obszaru
									G = G * regions.at<double>(regNum, 2) + Ga;
									R = R * regions.at<double>(regNum, 2) + Ra;
									S = S * regions.at<double>(regNum, 2) + Sa;
									regions.at<double>(regNum,2) += 1;
									G = G / regions.at<double>(regNum, 2);
									R = R / regions.at<double>(regNum, 2);
									S = S / regions.at<double>(regNum, 2);

									regions.at<double>(regNum,0) = G;
									regions.at<double>(regNum,1) = R;
									regions.at<double>(regNum,5) = S;
									if (nextInd < MAXL){
										pixList.at<int>(nextInd,0) = y1;
										pixList.at<int>(nextInd,1) = x1;
									}
									OutImg.at<int>(y1,x1) = regNum; //% pixel is "busy"
									nextInd = nextInd + 1;
								}
							}
						}
						listInd = listInd + 1;
						//std::cout << "listInd" << listInd << "\n";
						//%[ listInd, nextInd, regNum, x, y ]
					}
					//std::cout << "TEST regions regNum ? regions.rows" << regNum << ";" << regions.rows << "\n";
					////% środek masy obszaru
					ROI = pixList(cv::Rect(0,0,1,listInd -1)).clone();
					regions.at<double>(regNum,3) = cv::mean(ROI,cv::noArray()).val[0]; //%cy,
					//ROI.release();
					ROI = pixList(cv::Rect(1, 0, 1, listInd - 1)).clone();
					regions.at<double>(regNum,4) = cv::mean(ROI,cv::noArray()).val[0]; //% cx
					//ROI.release();
					//std::cout << "TEST regions regNum ? regions.rows" << regNum << ";"<< regions.rows << "\n";

					//%if regions(regNum,3) > 1
					//%    regions(regNum,1:2)

					//%input('?');

				}
			}
		//std::cout << "TEST out\n";
		pixList.release();
		
		// Zwróć wynik:
		// Niejawnie - macierz opisu obszarów
		if(regNum>0){ 
			regionDesc = regions(cv::Rect(0,0,regions.cols, regNum)).clone();
		}
		// Jawnie - obraz z rozmieszczeniem segmentów
		return OutImg;
	}


	//
	//
	// Funkcja pomocnicza 17: ClassImage1()
	//
	// Klasyfikacja obszarów w obrazie RGB
	//

	cv::Mat ClassImg;
	ROD::ResultOD wynikRGB = ROD::ResultOD(); // wynik klasyfikacji RGB

	int *ClassReg;
	int cryClass, crxClass;
	
	/*ROD::ResultOD*/void ClassImage1(int my, int mx, int maxG, int maxR, cv::Mat &MonoImg, 
		cv::Mat &Mask, cv::Mat &HistG, cv::Mat &HistR, cv::Mat &Segment, 
		int numReg, cv::Mat &Region, cv::Mat &roiSz, int typeA, ROD::UserParameters uParam)
	{
		ClassImg = cv::Mat::ones(my, mx, CV_8UC3);//uchar, 3 channels
		ClassReg = (int*)malloc(numReg * sizeof *ClassReg);
		for (int i = 0 ; i < numReg; ++i) ClassReg[i] = 0;

		std::vector<cv::Mat> ClassImg_planes;
		cv::split(ClassImg, ClassImg_planes);

		//%Region(:, 6)

		//% Inicjalizuj obiekt wynikowy
		for (int i = 0; i < 7; i++)
			wynikRGB.bel[i] = 0.0;
	
		cv::Mat roiSzyp;
		int GreenAppleRG, AppleRG, TailRG, SpotRG, YAppleRG1, YAppleRG2;

		//% Czy wykryto szypułkę ?
		//roiSz
		if (roiSz.at<int>(0, 0) == 0)
			roiSzyp = (cv::Mat_<int>(4, 1) << 0, 0, 0, 0); //% zdefiniuj wirtualne roi
		else
			roiSzyp = roiSz.clone(); //% dla filtracji obszarów "podejrzanych o defekty"

		//% Klasy:
		//% 1 : czerwone jabłko(brak defektu)
		//% 2 : defekt : plamka brązowa(spot)
		//% 3 : odcisk lekko szary
		//% 4 : porażenie grzybowe
		//% 5 : obszar szary przy szypułce(jeśli w obszarze szypułki)
		//% 6, 7 : wgniecenia(rozpatrywane w IR)
		//% 8 --> 1: zielone jabłko(brak defektu)

		//% Apple - minG, maxR
		//% Tail - maxG, minR
		//% Spot - average G, average R

		//%switch uParam.type
		switch (typeA)
		{
		case 1: //% red apple
			AppleRG = maxR - maxG - 12; //% ok. 70 - 15 = 55; ("zielone")
			//% AppleRG = 70 - 12; % ok. 70 - 15 = 55; ("zielone")
			SpotRG = AppleRG - 15; //% ok. > 40 brązowa plamka("żółte")
			TailRG = SpotRG - 10; //% ok > 30 lekko szary odcisk("szare")
			//% > 15 < 30 "porażenie grzybowe" ("czerwone") lub STD > 240
			GreenAppleRG = -5; //% ok < 0
			YAppleRG1 = 15; //% szary obszar przy szypułce
			YAppleRG2 = -5; //%szary obszar przy szypułce ?
			break;
		case 2: //% green apple
			//%GreenAppleRG = maxR - maxG + 4; % ok < 1
			GreenAppleRG = 5; //% ok < 1
			AppleRG = 70;
			TailRG = GreenAppleRG + 30;
			SpotRG = TailRG + 10;
			YAppleRG1 = 15;
			YAppleRG2 = 5;
			break;
		default:///*otherwise*/ %mixed, yellow apple
			//%GreenAppleRG = maxR - maxG + 5; % ok < 6
			GreenAppleRG = 5;
			AppleRG = 70; //% > 58
			TailRG = GreenAppleRG + 20; //% ok > 7 odcisk
			SpotRG = TailRG + 10; //% ok. > 15 plamka
			YAppleRG1 = 15;
			YAppleRG2 = 5;
			break;
		}

		//%
		int MONOTHRESH_ = 200;
		int STDRTHRESH_ = 220;
		int MAX_DEFEKT_SIZE = 1000;
		int MIN_DEFEKT_SIZE = 2;
		int MIN_SERIOUS_SIZE = 10;
		int MAX_SERIOUS_SIZE = 400;

		//%AppleG = maxG + 8
		//% TailG = AppleG + 8
		//% YAppleG = TailG + 6

		//% AppleR = maxR - 8
		//% SpotR = AppleR - 10
		//% YAppleR = SpotR - 10

		//% AppleGreen = maxGreen - 30
		double amax, bmax;
		cv::minMaxLoc(HistG, 0, &amax, 0, 0, cv::noArray());
		cv::minMaxLoc(HistR, 0, &bmax, 0, 0, cv::noArray());

		int numPix = 0;
		int ind, segInd, sizeR;
		double diff;

		for (int i = 0; i < my; i++)
		{
			for (int j = 0; j < mx; j++)
			{
				if (Mask.at<uchar>(i, j) == 0)
				{
					numPix = numPix + 1;
					//% na początek wszystko jest OK
					ClassImg_planes[0].at<uchar>(i, j) = 0; //%
					ClassImg_planes[1].at<uchar>(i, j) = 0; //%
					ClassImg_planes[2].at<uchar>(i, j) = 0; //%
					ind = 1;
					//% pixel classification
					//%diff = double(RImg(i, j)) - double(GImg(i, j));

					//% częstość danej wartości :
					//%a = double(HistG(GImg(i, j)));
					//%b = double(HistR(RImg(i, j)));

					//% ustal indeks obszaru dla piksela
					segInd = Segment.at<uchar>(i, j);

					sizeR = (int)Region.at<double>(segInd, 2); //% rozmiar

					//% jeśli region już został zaklasyfikowany to pomiń klasyfikację
					//% piksela
					if (ClassReg[segInd] > 0)
					{
						ind = ClassReg[segInd];
					}
					else
					{
						//% odstęp pomiędzy wartościami średnimi R i G dla obszaru
						diff = double(Region.at<double>(segInd, 1)) - Region.at<double>(segInd, 0);
						//% Klasyfikacja
						if ((diff > AppleRG) || (MonoImg.at<uchar>(i, j) > MONOTHRESH_) || (sizeR <= MIN_DEFEKT_SIZE))
						{
							ind = 1; //% czerwone / zielone jabłko
						}
						else
						{
							if (diff < GreenAppleRG) //% zielone jabłko
								ind = 1;
							else
							{
								if (diff > SpotRG)
								{
									if (sizeR < MAX_DEFEKT_SIZE) //% brązowa plamka
										ind = 2; //% defekt - brązowa "plamka"
									else
										ind = 1; //% za duża brązowa plamka - zółte jabłko bez defektu
								}
								else
								{
									if (diff > TailRG)
									{
										if (sizeR < MAX_DEFEKT_SIZE) //% grey - wgniecenie
											ind = 3; //% defekt "odcisk"
										else
											ind = 1;
									}
									else
									{
										if (sizeR < MAX_DEFEKT_SIZE) //%
										if ((sizeR > MIN_SERIOUS_SIZE) && (sizeR < MAX_SERIOUS_SIZE))
										{
											if (diff > YAppleRG1)
												ind = 4; //% szary obszar - porażenie grzybowe
											else
												ind = 5; //% brzeg lub przy szypułce
										}
									}
								}
							}
						}


						//% Korekta obszaru przy szypułce
						cryClass = (int)Region.at<double>(segInd, 3);// % środek cy
						crxClass = (int)Region.at<double>(segInd, 4);//% środek cx
						switch (ind)
						{
						case 2: //% defekty
						case 3:
						case 4:
							if ((crxClass > roiSzyp.at<int>(0, 0)) && (crxClass < roiSzyp.at<int>(2, 0)))
							{
								if ((cryClass > roiSzyp.at<int>(1, 0)) && (cryClass < roiSzyp.at<int>(3, 0)))
									ind = 5; //%szypułka
							}
							break;
						case 5: //% czy przy szypułce ?
							if ((crxClass <= roiSzyp.at<int>(0, 0)) || (crxClass >= roiSzyp.at<int>(2, 0)))
							{
								if ((cryClass <= roiSzyp.at<int>(1, 0)) || (cryClass >= roiSzyp.at<int>(3, 0)))
									ind = 1; //% to nie jest przy szypułce
							}
							break;
						default: //%{ 1, 6, 7 }, % brak defektu lub wgłębienie
							if (Region.at<double>(segInd, 5) > STDRTHRESH_) //% stdR
								ind = 4;
							break;
						}
						//% Skoryguj dużą zmienność
						if ((Region.at<double>(segInd, 5) > STDRTHRESH_) && (sizeR > MIN_DEFEKT_SIZE)) //% stdR
							ind = 4;
						//% ustaw klasę obszaru do którego należy piksel
						ClassReg[segInd] = ind;
					}
					//%Miara według odległości | CR - CG |
					//%ClassRedApple = abs(diff - AppleRG);
					//%ClassTail = abs(diff - TailRG);
					//%ClassSpot = abs(diff - SpotRG);
					//%ClassGrey = abs(diff - 0); % plamka szara
					//%ClassGreen = abs(diff + 30);
					//%Classes = [ClassRedApple ClassSpot ClassGrey ClassBlack ClassGreen];
					//%[val ind] = min(Classes);

					switch (ind)
					{
					case 1:
						ClassImg_planes[0].at<uchar>(i, j) = 0; //% zielone(czerwone lub zielone jabłko)
						ClassImg_planes[1].at<uchar>(i, j) = 255;
						ClassImg_planes[2].at<uchar>(i, j) = 0;
						break;
					case 2:
						//% brązowa plamka
						ClassImg_planes[2].at<uchar>(i, j) = 255; //% Defekt - plamka - żółty
						ClassImg_planes[1].at<uchar>(i, j) = 250;
						ClassImg_planes[0].at<uchar>(i, j) = 0;
						break;
					case 3:
						ClassImg_planes[2].at<uchar>(i, j) = 200; //% Defekt: lekko - szary odcisk
						ClassImg_planes[1].at<uchar>(i, j) = 200;
						ClassImg_planes[0].at<uchar>(i, j) = 128;
						break;
					case 4:
						ClassImg_planes[2].at<uchar>(i, j) = 200; //% Defekt: porażenie grzybowe - czerwone
						ClassImg_planes[1].at<uchar>(i, j) = 0;
						ClassImg_planes[0].at<uchar>(i, j) = 0;
						break;
					case 5:
						ClassImg_planes[2].at<uchar>(i, j) = 0; //% Obszar przy szypułce - niebieski
						ClassImg_planes[1].at<uchar>(i, j) = 0;
						ClassImg_planes[0].at<uchar>(i, j) = 255;
						break;
					default:
						ClassImg_planes[0].at<uchar>(i, j) = 255; //% nie powinno wystąpić
						ClassImg_planes[1].at<uchar>(i, j) = 255;
						ClassImg_planes[2].at<uchar>(i, j) = 255;
						break;
					}

					//%if (a < 200) && (b<200) % wartość rzadko występuje
					//%   if (diff >= SpotRG)
					//% ClassImg(i, j, :) = [0 255 0]; % spot brązowy
					//%   else
					//%       if diff >= TailRG
					//%          ClassImg(i, j, :) = [0 0 255]; %ogonek
					//%       end
					//%   end
					//%end

					//%if (diff <= YAppleRG1) && (diff >= YAppleRG2)
					//% ClassImg(i, j, :) = [0 255 0]; %plamka szara
					//%end
				}
			}
		}

		//% Statystyka defektów
		if (numPix > 0)
		{
			for (int i = 0; i < numReg; i++)
				switch (ClassReg[i])
			{
				case 1:
					wynikRGB.bel[0] = wynikRGB.bel[0] + Region.at<double>(i, 2); //% dodaj rozmiar
					break;
				case 2:
					wynikRGB.bel[1] = wynikRGB.bel[1] + Region.at<double>(i, 2); //% dodaj rozmiar
					break;
				case 3:
					wynikRGB.bel[2] = wynikRGB.bel[2] + Region.at<double>(i, 2); //% dodaj rozmiar
					break;
				case 4:
					wynikRGB.bel[3] = wynikRGB.bel[3] + Region.at<double>(i, 2); //% dodaj rozmiar
					break;
				case 5:
					wynikRGB.bel[4] = wynikRGB.bel[4] + Region.at<double>(i, 2); //% dodaj rozmiar
					break;
				case 6:
					wynikRGB.bel[5] = wynikRGB.bel[5] + Region.at<double>(i, 2); //% dodaj rozmiar
					break;
				default:
					wynikRGB.bel[6] = wynikRGB.bel[0] + Region.at<double>(i, 2); //% dodaj rozmiar
					break;
			}

			for (int i = 0; i < 5; i++)
			{
				wynikRGB.bel[i] = wynikRGB.bel[i] / double(numPix);
			}
			//%wynik.bel6 = wynik.bel6 / double(numPix);
			//%wynik.bel7 = wynik.bel7 / double(numPix);
		}

		for (int i = 0; i < my; i++)
			for (int j = 0; j < mx; j++)
				if (Mask.at<uchar>(i, j) != 0)//wyciecie czarnego tla
					for(int k=0;k<3;k++) 
						ClassImg_planes[k].at<uchar>(i, j)=255;

		cv::merge(ClassImg_planes, ClassImg);
		roiSzyp.release();

		//cv::imshow("TEST2", ClassImg);

		wynikRGB.pixNum = numPix;
		wynikRGB.classImage = ClassImg;
		wynikRGB.classReg = ClassReg;
		wynikRGB.numReg = numReg;
		wynikRGB.regions = Region;//zmienione na cv::Mat z double (*regions)[5]

		//return wynik;
	}

	//
	//
	// Funkcja pomocnicza 18: ClassIRImage()
	//

	ROD::ResultOD wynikIr = ROD::ResultOD();

	cv::Mat ClassIrImg;

	void ClassIRImage(int my, int  mx, int Cx, int Cy, cv::Mat &roiSz, cv::Mat &GreenImg,
		cv::Mat &Mask, cv::Mat &EdgeImg, cv::Mat &ClassImg)
	{
		ClassIrImg = 255 * cv::Mat::ones(my, mx, CV_8UC3);
		std::vector<cv::Mat> ClassIrImg_planes;
		cv::split(ClassIrImg, ClassIrImg_planes);
		cv::Mat roiSzypIr;

		if (roiSz.at<int>(0, 0) == 0)
		{
			roiSzypIr = (cv::Mat_<int>(4, 1) << 0, 0, 0, 0); //% zdefiniuj wirtualne roi
		}
		else
			roiSzypIr = roiSz.clone(); //% dla filtracji obszarów "podejrzanych o defekty"

		int numPix = 0;
		double bel1, bel2, bel3, bel4, bel5, bel6, bel7;
		bel1 = bel2 = bel3 = bel4 = bel5 = bel6 = bel7 = 0.0;
		//% Apple - minG, maxR
		//% Tail - maxG, minR
		//% Spot - average G, average R

		//%Cx = mx / 2;
		//%Cy = my / 2;
		
		std::vector<cv::Mat> ClassImg_planes;
		cv::split(ClassImg, ClassImg_planes);

		int MINVALUE = 90;
		int MAXVALUE = 200;
		double DecayParam = 0.7;
		int ind = 0, diff;
		double radius, thresh;

		for (int i = 0; i < my; i++)
		for (int j = 0; j < mx; j++)
		{

			ind = 0;
			if (Mask.at<uchar>(i, j) == 0)
			{
				//% na początek wszystko jest białe
				ClassIrImg_planes[0].at<uchar>(i, j) = 255; //%
				ClassIrImg_planes[1].at<uchar>(i, j) = 255;
				ClassIrImg_planes[2].at<uchar>(i, j) = 255;
				ind = 4;
				diff = GreenImg.at<uchar>(i, j);
				//% pixel classification
				radius = abs(j - Cx - 30) + 0.7 * abs(i - Cy);
				if (radius <= 40) //% central enlighted part
				{
					if (diff > MAXVALUE)
					{
						ClassIrImg_planes[0].at<uchar>(i, j) = 0; //%OK
						ClassIrImg_planes[1].at<uchar>(i, j) = 255;
						ClassIrImg_planes[2].at<uchar>(i, j) = 0;
						ind = 1;
					}
					else
					{
						//% wgniecenia - ciemne/czarne
						
						ind = 2;
					}
				}
				else
				{
					thresh = MAXVALUE - 1.2 * (radius - 40);
					if (diff > thresh)
					{
						//% OK - zielone
						ClassIrImg_planes[0].at<uchar>(i, j) = 0;
						ClassIrImg_planes[1].at<uchar>(i, j) = 255;
						ClassIrImg_planes[2].at<uchar>(i, j) = 0;
						ind = 1;
					}
					else
					{
						//% wgniecenie - ciemne/czarne
						
						ind = 2;
					}
				}

				//% Czy w obszarze szypułki
				if ((j > roiSzypIr.at<int>(0, 0)) && (j < roiSzypIr.at<int>(2, 0)))
				if ((i > roiSzypIr.at<int>(1, 0)) && (i < roiSzypIr.at<int>(3, 0)))
				{
					//% szypułka
					ClassIrImg_planes[0].at<uchar>(i, j) = 250;
					ClassIrImg_planes[1].at<uchar>(i, j) = 50;
					ClassIrImg_planes[2].at<uchar>(i, j) = 50;
					ind = 4;
				}
				numPix = numPix + 1;
			}
			//% w zasadzie brzeg jabłka nas nie interesuje
			if (Mask.at<uchar>(i, j) == 2)
			{
				//% brzeg
				ClassIrImg_planes[2].at<uchar>(i, j) = 10;
				ClassIrImg_planes[1].at<uchar>(i, j) = 20;
				ClassIrImg_planes[0].at<uchar>(i, j) = 255;
				ind = 5;
				numPix = numPix + 1;
			}

			if (ind == 1) //% jest wypukłe OK, ale może być plamka
			{
				//%sprawdź krawędzie
				if (EdgeImg.at<uchar>(i, j) > 0)
				{
					ClassIrImg_planes[2].at<uchar>(i, j) = 255;
					ClassIrImg_planes[1].at<uchar>(i, j) = 250;
					ClassIrImg_planes[0].at<uchar>(i, j) = 5;
					ind = 3;
					// plamka - szara
					
				}
			}

			switch (ind)
			{
			case 1: //% jabłko dobre
				bel1 += 1;
				break;
			case 2: //% wgniecenie
				bel2 += 1;
				if (ClassImg_planes[1].at<uchar>(i, j) == 255) // bez zmiany koloru
				{	bel6 +=1;
					ClassIrImg_planes[0].at<uchar>(i, j) = 50;
					ClassIrImg_planes[1].at<uchar>(i, j) = 50;
					ClassIrImg_planes[2].at<uchar>(i, j) = 50;
				}	
				else
				{
					bel7 +=1;
					// wgniecenia - czarne
					ClassIrImg_planes[0].at<uchar>(i, j) = 10;
					ClassIrImg_planes[1].at<uchar>(i, j) = 10;
					ClassIrImg_planes[2].at<uchar>(i, j) = 10;
				}	

				break;
			case 3: //% plamki
				bel3 += 1;
				break;
			case 4: //% szypułka
				bel4 += 1;
				break;
			case 5: //% brzeg
				bel5 += 1;
				break;
			default: //brzeg
				break;
			}
		}
		
		//% Statystyka defektów
		//wynikIr
		int numPix1 = bel1 + bel2 + bel3 + bel4 + bel5;
		wynikIr.pixNum = numPix1;
		if (numPix1 > 0)
		{
			wynikIr.bel[0] = (bel1 + bel5) / numPix1;
			wynikIr.bel[1] = bel2 / numPix1;
			wynikIr.bel[2] = bel3 / numPix1;
			wynikIr.bel[3] = bel4 / numPix1;
			wynikIr.bel[4] = 0;
			wynikIr.bel[5] = bel6 / double(numPix1);
			wynikIr.bel[6] = bel7 / double(numPix1);
		}

		for (int i = 0; i < my; i++)
			for (int j = 0; j < mx; j++)
				if (Mask.at<uchar>(i,j)==1)
					for (int k = 0; k < 3; k++)
						ClassIrImg_planes[k].at<uchar>(i, j) = 255;
		
		cv::merge(ClassIrImg_planes, ClassIrImg);
		
		//cv::imshow("TEST", ClassIrImg);
		
		wynikIr.pixNum = round(numPix1);
		wynikIr.classImage = ClassIrImg;
		wynikIr.classReg = 0;
		wynikIr.numReg = 0;
		wynikIr.regions = cv::Mat();
		roiSzypIr.release();
	}


	//
	//
	// Funkcja pomocnicza 19: ScalWyniki()
	//

	ROD::ResultOD ScalWyniki(int my, int  mx, ROD::UserParameters userParam, cv::Mat &Mask, cv::Mat &classImg, cv::Mat &classIRImg, 
		cv::Mat &class3DMap, ROD::ResultOD &wynikRGB, ROD::ResultOD &wynikIR, ROD::ResultOD &result)
	{
		ROD::ResultOD wynik;
		//
		std::vector<cv::Mat> ClassImg_planes;
		cv::split(classImg, ClassImg_planes);
		
		std::vector<cv::Mat> ClassIRImg_planes;
		cv::split(classIRImg, ClassIRImg_planes);

		// Przesunięcie pomiędzy obrazem IR a RGB
		// To powinien być parametr w UserParameters
		int DX = -5;
		int DY = 5;
		int begx = -DX;
		int endy = my - DY;

		// Zasadnicza informacja jest już w wynikRGB
		wynik = wynikRGB; 
		// Informacja o ogonku i szypułce jest w result
		wynik.jestOgon = result.jestOgon;
		wynik.ogonCx = result.ogonCx; // położenie nasady szypułki
		wynik.ogonCy = result.ogonCy;
		wynik.cx = result.cx ; // Uwaga: w układzie obrazu !
		wynik.cy = result.cy; // --" --
		// Promień okręgu aproksymującego kontur
		wynik.radius = result.radius;
		
		// Przejrzyj klasyfikowane obrazy
		int iy, jx;
		if (wynikIR.pixNum > 0) // "Nałóż" klasy dla obrazu IR - analiza uproszczona 2D
		{
			for (int i = 0; i < endy; i++)
				for (int j = begx; j < mx; j++)
				{ 
					if (Mask.at<uchar>(i, j) == 0)
					{
						jx = j + DX; iy = i + DY;
						if (ClassIRImg_planes[0].at<uchar>(iy, jx) != 255) // jest defekt
						{ 
							ClassImg_planes[0].at<uchar>(i, j) = ClassIRImg_planes[0].at<uchar>(iy, jx) ;
							ClassImg_planes[1].at<uchar>(i, j) = ClassIRImg_planes[1].at<uchar>(iy, jx) ;
							ClassImg_planes[2].at<uchar>(i, j) = ClassIRImg_planes[2].at<uchar>(iy, jx) ;
						}
					}
				}

			double ratio = ((double)wynikIR.pixNum) / wynik.pixNum; 
			wynik.bel[5] = wynikIR.bel[5] * ratio;
			wynik.bel[6] = wynikIR.bel[6] * ratio;
			wynik.bel[0] = 1.0 - (wynik.bel[1] + wynik.bel[2] + wynik.bel[3] + 
				wynik.bel[4] + wynik.bel[5] + wynik.bel[6]); 
		}

		cv::merge(ClassImg_planes, wynik.classImage);

		return wynik;
	}

}// obszar ROD
#endif
