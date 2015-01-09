// Plik rod_KonturRGB.cpp
	//
	//
	// Funkcja pomocnicza 6: KonturRGB()
	//

// TO DO: to powinny być parametry funkcji
	
	//::cv::Mat Kontur = ::cv::Mat::zeros(720, 2, CV_64FC1); // element co pewien stopień kątowy
	//::cv::Mat Kontur1D = ::cv::Mat::zeros(720, 1, CV_64FC1);
	//::cv::Mat Grad1D = ::cv::Mat::zeros(720, 1, CV_64FC1);
	const double PI = CV_PI; //std::atan(1.0)*4;

	void KonturRGB(double &MINVALUE, double &MINREDVALUE, int DIRS, int& Cx, int& Cy, int Radius,
		::cv::Mat &src_col, ::cv::Mat &src_mono, ::cv::Mat &Kontur, ::cv::Mat &Kontur1D, ::cv::Mat &Grad1D  )
	{
	  // Wyznacza kontur obiektu w obrazie RGB oraz środek masy, (Cx, Cy)
	 
	  int mx, my, k;
	  double HalfX, HalfY, QuatX, Quat3X;
	  double delrad, alfa;
	  double x, y, x0, y0, x1, y1, x2, y2, x3, y3, x4, y4, nx, ny;
	  double rr, dx, dy, rr0, rr1, dr;
	  // dla wygładzania konturu
	  double meanRadius;
	  int radElNum; 
	  
	  int yr0, xr0, yr1, xr1, yr2, xr2; // wewnątrz konturu
	  int yl0, xl0, yl1, xl1, yl2, xl2; // zewnątrz konturu

	  double r11, r12, r13, p11, p12, p13;
	  int DIRS1, DIRS2; // DIRS = liczba badanych kierunków = elementów konturu
	  int startInd;
	  int i0, j0; int jestN, iend;

	  my = src_col.rows;
	  mx = src_col.cols;
	  int MINRADIUS = (int)(0.80 * Radius); // zakładany minimalny promień
	  int MAXRAD = (int)(1.20 * Radius); // maksymalny promień

	  //%CONSTANTS
	  //%MINVALUE = 25;
	  //%MINREDVALUE = 42;

	  // Stałe pomocnicze
	  HalfX = mx / 2;
	  QuatX = mx / 4;
	  Quat3X = QuatX + HalfX;
	  HalfY = my / 2;

	  // ogranicz promień jabłka
	   if (MAXRAD > HalfY) MAXRAD = HalfY;

	  std::vector<cv::Mat> src_col_planes(3);
	  ::cv::Mat jestKontur = ::cv::Mat::zeros(DIRS,1, CV_8UC1); //% czy jest prawidłowa dana?
	  ::cv::Mat sinalfa = ::cv::Mat::zeros(DIRS,1, CV_64FC1);
	  ::cv::Mat cosalfa = ::cv::Mat::zeros(DIRS,1, CV_64FC1);

	  ::cv::Mat MatTmp = src_col;
	  ::cv::split(/*src_col*/MatTmp, src_col_planes);

	  // Wyznacz kontur
	  delrad = 2.0 * PI / DIRS;
	  //%
	  alfa = -PI;
	  meanRadius = Radius;
	  radElNum = 0;
	  int r, r1, jest, jest1;
	  for (int i=0; i<DIRS; i++)
	  {
	      alfa += delrad;
	      sinalfa.at<double>(i,0) = sin(alfa);
	      cosalfa.at<double>(i,0) = cos(alfa);
		  
	      //for (r = MAXRAD-1; r>= MINRADIUS; r--)
		  jest = 0;
		  for (r = MINRADIUS + 1; r< MAXRAD; r++)
		  {
	          x = double(Cx) + (r+1) * cosalfa.at<double>(i,0);
	          y = -((r+1) * sinalfa.at<double>(i,0) - double(Cy));
	          yr0 = round(y);
	          xr0 = round(x);
	          x = double(Cx) + r * cosalfa.at<double>(i,0);
	          y = -(r * sinalfa.at<double>(i,0) - double(Cy));
	          yr1 = round(y);
	          xr1 = round(x);
	          x = double(Cx) + (r-1) * cosalfa.at<double>(i,0);
	          y = -((r-1) * sinalfa.at<double>(i,0) - double(Cy));
	          yr2 = round(y);
	          xr2 = round(x);

	          if ( (xr0 >= 0) && (xr0 < mx) )
	            if ( (yr0 >=0) && (yr0 < my) )
	              {
	                Kontur.at<double>(i,0) = double(xr0);
	                Kontur.at<double>(i,1) = double(yr0);

	                //%y = - (tgalfa * (i - Cx) - Cy); % use image coordinates but change the
	                //% sign of the result into memory index
	                r11 = src_col_planes[2].at<uchar>(yr0,xr0);//yr0,xr0); Red is [0] or [2] ??
	                r12 = src_col_planes[2].at<uchar>(yr1,xr1);//yr1,xr1);
	                r13 = src_col_planes[2].at<uchar>(yr2,xr2);//yr2,xr2);
	                p11 = src_mono.at<uchar>(yr0,xr0);//yr0,xr0);
	                p12 = src_mono.at<uchar>(yr1,xr1);//yr1,xr1);
	                p13 = src_mono.at<uchar>(yr2,xr2);//yr2,xr2);
	                if ( ((p11 < MINVALUE) || ( r11 < MINREDVALUE)) && ((p12 <= MINVALUE) || (r12 <= MINREDVALUE)) )
	                //if ( (p11 > MINVALUE) && ( r11 > MINREDVALUE) && (p12 > MINVALUE) && (r12 >  MINREDVALUE) )
					{
						if ( (r < (int)(0.90 * meanRadius)) && (radElNum > 1))
						//if ( ((r-1) > (int)(1.21 * meanRadius)) && (radElNum > 2))
							meanRadius = meanRadius; // nic nie rób
						else {
							jest = 1;
							break; 
						}
	                }
	            }
		  }

		  // Od zewnątrz
		  jest1 = 0;
		  for (r1 = MAXRAD-1; r1 >= MINRADIUS; r1--)
		  {
	          x = double(Cx) + (r1+1) * cosalfa.at<double>(i,0);
	          y = -((r1+1) * sinalfa.at<double>(i,0) - double(Cy));
	          yl0 = round(y);
	          xl0 = round(x);
	          x = double(Cx) + r1 * cosalfa.at<double>(i,0);
	          y = -(r1 * sinalfa.at<double>(i,0) - double(Cy));
	          yl1 = round(y);
	          xl1 = round(x);
	          x = double(Cx) + (r1-1) * cosalfa.at<double>(i,0);
	          y = -((r1-1) * sinalfa.at<double>(i,0) - double(Cy));
	          yl2 = round(y);
	          xl2 = round(x);

	          if ( (xl0 >= 0) && (xl0 < mx) )
	            if ( (yl0 >=0) && (yl0 < my) )
	              {
	                // 
	                r11 = src_col_planes[2].at<uchar>(yl0,xl0);//yr0,xr0); Red is [2] ?!
	                r12 = src_col_planes[2].at<uchar>(yl1,xl1);//yr1,xr1);
	                r13 = src_col_planes[2].at<uchar>(yl2,xl2);//yr2,xr2);
	                p11 = src_mono.at<uchar>(yl0,xl0);//yr0,xr0);
	                p12 = src_mono.at<uchar>(yl1,xl1);//yr1,xr1);
	                p13 = src_mono.at<uchar>(yl2,xl2);//yr2,xr2);

	                //if ( ((p11 < MINVALUE) || ( r11 < MINREDVALUE)) && ((p12 <= MINVALUE) || (r12 <= MINREDVALUE)) )
	                if ( (p11 > MINVALUE) && ( r11 > MINREDVALUE) && (p12 > MINVALUE) && (r12 >  MINREDVALUE) )
	                {
						
						if ( (r1 > (int)(1.1 * meanRadius)) && (radElNum > 1))
						//if ( ((r-1) > (int)(1.21 * meanRadius)) && (radElNum > 2))
							meanRadius = meanRadius; // nic nie rób
						else {
							jest1 = 1;
							break;
						}
					}
				}
		  }

		  // 
		  if ((jest == 1) && (jest1 == 1)) // są oba wyniki
		  {
			  Kontur.at<double>(i,0) = (xr2 + xl2) * 0.5;
			  Kontur.at<double>(i,1) = (yr2 + yl2) * 0.5;
			  jestKontur.at<uchar>(i,0) = 1;
							
			  meanRadius = 0.7 * meanRadius + 0.15 * (r + r1);
			  radElNum ++;
		  }
		  //TEST
		  //if (i < 200)
		  //cout << "i=" << i << ": r=" << r << ", (x, y)="<< x << "," << y << ", (r11, p11)=" << r11 <<", "<< p11<< endl;
		  //
	  }
	  
	  //::cv::merge(src_col_planes,src_col);

	  //% Wygładzanie konturu
	  x0 = Kontur.at<double>(0,0);
	  y0 = Kontur.at<double>(0,1);
	  for (int i= DIRS-1; i>=0; i-=1)
	    if (jestKontur.at<uchar>(i,0) == 1)
	      {
	        x0 = Kontur.at<double>(i,0);
	        y0 = Kontur.at<double>(i,1);
	        break;
	      }
	  DIRS1 = DIRS - 1;
	  for (int i=0;i<DIRS1; i+=1)
	    {
	      x = x0;
	      y = y0;
	      if (jestKontur.at<uchar>(i,0) == 1)
	        {
	          x = Kontur.at<double>(i,0);
	          y = Kontur.at<double>(i,1);
	          k =i;
	          break;
	        }
	    }
	  j0 = k+1; //j0 = i+1;
	  for (int j=j0;j<DIRS;j+=1)
	    {
	      x1 = x;
	      y1 = y;
	      if (jestKontur.at<uchar>(j,0) == 1)
	        {
	          x1 = Kontur.at<double>(j,0);
	          y1 = Kontur.at<double>(j,1);
	          //%wygładzanie
	          nx = round(0.3333 *(x0 + x + x1));
	          ny = round(0.3333 *(y0 + y + y1));
	          Kontur.at<double>(k, 0) = nx;
	          Kontur.at<double>(k, 1) = ny;
	          k = j;
	          x0 = nx;
	          y0 = ny;
	          x = x1;
	          y = y1;
	        }
	    }

	  //% Aproksymuj brakujące pomiary
	  startInd = -1;
	  for (int i=0;i<DIRS1;i++)
	    if (jestKontur.at<uchar>(i,0) == 1)
	      {
	        startInd = i;
	        dx = Kontur.at<double>(startInd, 0) - double(Cx);
	        dy = -(Kontur.at<double>(startInd, 1) - double(Cy));
	        rr0 = sqrt(dx*dx + dy*dy);
	        break;
	      }

	  if (startInd > -1)
	    {
	      rr = rr0;
	      i0 = startInd+1;
	      for (int i= i0; i< DIRS;i++)
	        {
	          if (jestKontur.at<uchar>(i,0) == 0)
	            {
	              //% znajd nastêpny
	              jestN=0;
	              for (int j=i; j<DIRS; j++)
	                if (jestKontur.at<uchar>(j,0) == 1)
	                  {
	                    jestN=1;
	                    dx = Kontur.at<double>(j, 0) - double(Cx);
	                    dy = -(Kontur.at<double>(j, 1) - double(Cy));
	                    rr1 = sqrt(dx*dx + dy*dy);
	                    dr = (rr1 - rr) /(j-i);
	                    break;
	                  }
	              //%aproksymuj
	              if (jestN==1)
	                rr= rr+dr;
	              //%aproksymuj
	              x = double(Cx) + rr * cosalfa.at<double>(i,0);
	              y = -(rr * sinalfa.at<double>(i,0) - double(Cy) );
	              yr0 = round(y);
	              xr0 = round(x);
	              Kontur.at<double>(i,0) = double(xr0);
	              Kontur.at<double>(i,1) = double(yr0);
	              jestKontur.at<uchar>(i,0) = 1;
	            }
	          else
	            {
	              //%new r
	              dx = Kontur.at<double>(i, 0) - double(Cx);
	              dy = -(Kontur.at<double>(i, 1) - double(Cy));
	              rr = sqrt(dx*dx + dy*dy);
	            }
	        }
	      if (startInd > 1)
	        {
	          //%aproksymuj pocz¹tek
	          rr1= rr;
	          dr = (rr0 - rr1) /startInd;
	          iend = startInd - 1;
	          for (int i=0; i<iend; i++)
	            {
	              rr = rr + dr;
	              x = double(Cx) + rr * cosalfa.at<double>(i,0);
	              y = -(rr * sinalfa.at<double>(i,0) - double(Cy));
	              yr0 = round(y);
	              xr0 = round(x);
	              Kontur.at<double>(i,0) = double(xr0);
	              Kontur.at<double>(i,1) = double(yr0);
	              jestKontur.at<uchar>(i,0) = 1;

	            }
	        }
	    }

	  //% Oblicz srodek masy konturu
	  Cx = round( cv::mean(Kontur.col(0)).val[0] );
	  Cy = round( cv::mean(Kontur.col(1)).val[0] );


	  //% Końcowe wygładzanie konturu 5-elementową średnią
	  //% Dla indeksu DIRS
	  x0 = Kontur.at<double>(DIRS - 3,0); y0 = Kontur.at<double>(DIRS - 3,1);
	  x1 = Kontur.at<double>(DIRS -2,0); y1 = Kontur.at<double>(DIRS -2,1);
	  x2 = Kontur.at<double>(DIRS-1,0); y2 = Kontur.at<double>(DIRS-1,1);
	  x3 = Kontur.at<double>(0,0); y3 = Kontur.at<double>(0,1);
	  x4 = Kontur.at<double>(1,0); y4 = Kontur.at<double>(1,1);
	  x = 0.2 * (x0+x1+x2+x3+x4);
	  y = 0.2 * (y0+y1+y2+y3+y4);
	  Kontur.at<double>(DIRS-1, 0) = round(x);
	  Kontur.at<double>(DIRS-1, 1) = round(y);

	  DIRS2 = DIRS - 2;
	  for (int i=0; i<DIRS2; i++)
	    {
	      x0 = x1; y0 = y1;
	      x1 = x2; y1 = y2;
	      x2 = x3; y2 = y3;
	      x3 = x4; y3 = y4;
	      x4 = Kontur.at<double>(i+2,0);
	      y4 = Kontur.at<double>(i+2,1);
	      x = 0.2 * (x0+x1+x2+x3+x4);
	      y = 0.2 * (y0+y1+y2+y3+y4);
	      Kontur.at<double>(i, 0) = round(x);
	      Kontur.at<double>(i, 1) = round(y);
	 //
		  //if(round(x)<0)Kontur.at<double>(i, 0)=0;
		  //if(round(y)<0)Kontur.at<double>(i, 1)=0;
	      //if(round(x)<0||round(y)<0)std::cout<<"\n\nX || Y < 0 (~671)\n"<<round(x)<<"\n"<<round(y)<<"\ni: "<<i<<"\n";
	    }
	  //% Dla DIRS - 1
	  x0 = x1; y0 = y1;
	  x1 = x2; y1 = y2;
	  x2 = x3; y2 = y3;
	  x3 = x4; y3 = y4;
	  x4 = Kontur.at<double>(0,0);
	  y4 = Kontur.at<double>(0,1);
	  x = 0.2 * (x0+x1+x2+x3+x4);
	  y = 0.2 * (y0+y1+y2+y3+y4);
	  Kontur.at<double>(DIRS1-1, 0) = round(x);
	  Kontur.at<double>(DIRS1-1, 1) = round(y);

	  // Wyznacz funkcję 1D odległości punktów konturu od środka masy
	  for (int i=0; i<DIRS; i++)
	    {
	      dx = Kontur.at<double>(i, 0) - double(Cx);
	      dy = -(Kontur.at<double>(i, 1) - double(Cy));
	      Kontur1D.at<double>(i,0) = sqrt(dx*dx + dy*dy);
	    }

	  // Wyznacz gradient funkcji 1D odległości konturu
	  Grad1D.at<double>(0,0) = Kontur1D.at<double>(0,0) - Kontur1D.at<double>(DIRS-1,0);
	  for (int i=1; i< DIRS; i++)
	    Grad1D.at<double>(i,0) = Kontur1D.at<double>(i,0) - Kontur1D.at<double>(i-1,0);

	  jestKontur.release();
	  sinalfa.release();
	  cosalfa.release();

	}
