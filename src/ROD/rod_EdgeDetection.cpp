// Plik w funkcją ROD::EdgeDetection()

/**
	* Funkcja pomocnicza 1: EdgeDetection()
	* Dla obrazu mono generuje 2 obrazy krawędziowe (siła i kierunek krawędzi) ,
	* Następnie siła krawędzi obcięta zostaje do zakresu <theta * Max_sila, 255>
	*/
	void EdgeDetection( int lowThresh, double theta, ::cv::Mat &InImg, ::cv::Mat &EdgeImg, ::cv::Mat &DirImg)
	{
		int my = InImg.rows;
		int mx = InImg.cols;

		::cv::Mat DXImg, DYImg;
		::cv::Mat Xkernel = ::cv::Mat(::cv::Matx33f(-1.0, 0.0, 1.0,
			-2.0, 0.0, 2.0, 
			-1.0,  0.0,  1.0)) ;
		::cv::Mat Ykernel = ::cv::Mat(::cv::Matx33f(-1.0, -2.0, -1.0, 
			0.0, 0.0, 0.0, 
			1.0,  2.0,  1.0));
		
		filter2D(InImg, DXImg, CV_32F, Xkernel, ::cv::Point(-1,-1));
		filter2D(InImg, DYImg, CV_32F, Ykernel, ::cv::Point(-1,-1));

		::cv::Mat MImg = abs(DXImg) + abs(DYImg); // amplituda kraw�dzi
		// znajd� warto�� maksymaln�
		double minVal, maxVal;
		::cv::minMaxIdx(MImg, &minVal, &maxVal, 0, 0, ::cv::noArray());
		int aThresh = int(theta * maxVal); // 

		// std::cout << "aThresh = " << aThresh << std::endl;

		// Pocienianie ze sta�ym i adaptacyjnym progiem
		if (aThresh < lowThresh)
			aThresh = lowThresh;

		for( int y = 0; y < my; y++ )
		{
			float* Mptr = MImg.ptr<float>(y);

			for( int x = 0; x < mx; x++ )
			{
				float& Dx = DXImg.at<float>(y, x);
				float& Dy = DYImg.at<float>(y, x);

				if (Mptr[x] < aThresh) // eliminuj niestotne kraw�dzie
				{
					Mptr[x] = 0;
					Dx = 0;
				}
				else 
				{
					Dx = atan2(Dy, Dx);
					if (Mptr[x] > 255)
						Mptr[x] = 255;
				}
			}
		}

		// Wynikowe obrazy
		MImg.convertTo(EdgeImg,CV_8U); // amplituda kraw�dzi
		DirImg = DXImg.clone(); // kierunke kraw�dzi

		DXImg.release();
		DYImg.release();
		Xkernel.release();
		Ykernel.release();
		MImg.release();
		
	}

