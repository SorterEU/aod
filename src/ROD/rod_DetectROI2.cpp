// Plik rod_DetectROI2.cpp

	//
	// Funkcja pomocnicza 5: DetectROI2()
	//

	
	void DetectROI2(cv::Mat &src2, double cornerMax, double cornerMin, int bbox[])
	{
		double IRcolumnMin, IRcolumnMax, IRrowMin, IRrowMax;
		int gy = src2.rows;
		int gx = src2.cols;
		cv::Mat columnJ = cv::Mat::zeros(gy, 1, CV_32FC1);
		cv::Mat rowI = cv::Mat::zeros(gx, 1, CV_32FC1);
		double absCornerKernel;
		cv::Mat cornerKernelLeft, cornerKernelRight;
		cornerKernelLeft = (cv::Mat_<float>(14,1) <<-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,0.05,0.25,0.5,0.75,1.0);
		cornerKernelRight = (cv::Mat_<float>(14,1) <<1.0, 0.75, 0.5, 0.25, 0.05, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1);
		//double cornerKernelLeft[14] = {-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,0.05,0.25,0.5,0.75,1.0};
		//double cornerKernelRight[14] = {1.0, 0.75, 0.5, 0.25, 0.05, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1};
		//::cv::Mat columnJGrad, rowIGrad;
		//rowI.copyTo(rowIGrad);
		//columnJ.copyTo(columnJGrad);
		double maxColumn, maxRow, threshMax;
		int k;
		double minJ, maxJ;
		//cv::Mat tempMat;


		absCornerKernel = 0;
		absCornerKernel= cv::sum( abs(cornerKernelLeft) ).val[0];
		for (int j=0;j<gx;j++)
			columnJ.at<float>(j,0)=cv::sum(src2.col(j)).val[0];
		for (int i=0;i<gy;i++)
			rowI.at<float>(i,0)=cv::sum(src2.row(i)).val[0];
		//%row search
		::cv::minMaxLoc(rowI,0, &maxRow, 0, 0, ::cv::Mat());
		threshMax = cornerMax * maxRow;


		k=10;
		for (int j= round(gy/3.0)-1;j>=9;j-=1)
			if (rowI.at<float>(j,0) < float(threshMax))
			{
				k = j;
				break;
			}

		IRrowMin = 10;
		double absRowCurrent;
		double corrFactor;
		cv::Mat rowCurrent;
		//cv::Mat Transposed;
		//for(int i=0;i<14;i++)
		//	tempMat.push_back(cornerKernelLeft[i]);
		for (int i = k-1; i>=9; i-=1)
		{
			for(int l=9;l>=-4;l--)
				rowCurrent.push_back( rowI.row(i-l) );
			::cv::minMaxLoc(rowCurrent,&minJ, &maxJ, 0, 0, cv::Mat());
			rowCurrent = (rowCurrent - minJ) / (0.9*(maxJ - minJ)) - 0.1; //%normalization
			//%correlation with kernel
			absRowCurrent = cv::sum(abs(rowCurrent)).val[0];
			//cv::transpose(rowCurrent,Transposed);
			corrFactor = cv::sum(cornerKernelLeft*rowCurrent.t()).val[0]/(absCornerKernel * absRowCurrent);//
			if (corrFactor >= cornerMin)
			{
				IRrowMin = i;
				break;
			}
			rowCurrent.pop_back(rowCurrent.rows);
		}
		k=gy-9;
		for (int j= round(gy*2.0/3.0)-1; j<gy-9;j++)
			if (rowI.at<float>(j,0) < float(threshMax))
			{
				k = j;
				break;
			}

		IRrowMax = gy-9;
		//tempMat.pop_back(tempMat.rows);
		//for(int i=0;i<14;i++)
		//	tempMat.push_back(cornerKernelRight[i]);
		for (int i = k-1; i<gy-9;i++)
		{
			for(int l=4;l>=-9;l--)
				rowCurrent.push_back( rowI.row(i-l) );
			::cv::minMaxLoc(rowCurrent,&minJ, &maxJ, 0, 0, cv::Mat());
			rowCurrent = (rowCurrent - minJ) / (0.9*(maxJ - minJ)) - 0.1; //%normalization
			//%correlation with kernel
			absRowCurrent = cv::sum(abs(rowCurrent)).val[0];
			//cv::transpose(rowCurrent,Transposed);
			corrFactor = cv::sum(cornerKernelRight*rowCurrent.t()).val[0]/(absCornerKernel * absRowCurrent);
			if (corrFactor >= cornerMin)
			{
				IRrowMax = i;
				break;
			}
			rowCurrent.pop_back(rowCurrent.rows);
		}

		//% column search
		::cv::minMaxLoc(columnJ, 0, &maxColumn, 0, 0, cv::Mat());
		//threshMax = cornerMax * maxRow; //b��d ??
		threshMax = cornerMax * maxColumn; // poprawione - ?

		double absColCurrent;
		cv::Mat colCurrent;
		k=10;
		for (int j= round(gx/3.0)-1;j>=9;j--)
			if (columnJ.at<float>(j,0) < float(threshMax))
			{
				k = j;
				break;
			}
		IRcolumnMin = 10;
		//tempMat.pop_back(tempMat.rows);
		//for(int i=0;i<14;i++)
		//	tempMat.push_back(cornerKernelLeft[i]);
		for (int i = k-1; i>=9; i--)
		{
			for(int l=9;l>=-4;l--)
				colCurrent.push_back( rowI.row(i-l) );
			::cv::minMaxLoc(colCurrent, &minJ, &maxJ, 0, 0, cv::Mat());
			colCurrent = (colCurrent - minJ) / (0.9*(maxJ - minJ)) - 0.1; //%normalization
			//%correlation with kernel
			absColCurrent = cv::sum(abs(colCurrent)).val[0];
			//cv::transpose(colCurrent,Transposed);
			corrFactor = cv::sum(cornerKernelLeft  * colCurrent.t()).val[0]/(absCornerKernel * absColCurrent);
			if (corrFactor >= cornerMin)
			{
				IRcolumnMin = i;
				break;
			}
			colCurrent.pop_back(colCurrent.rows);
		}

		k=gx-9;
		for (int j= round(gx*2.0/3.0)-1; j<gx-9;j++)
			if (columnJ.at<float>(j,0) < float(threshMax))
			{
				k = j;
				break;
			}

		IRcolumnMax = gx-9;
		//tempMat.pop_back(tempMat.rows);
		//for(int i=0;i<14;i++)
		//	tempMat.push_back(cornerKernelRight[i]);
		for (int i = k-1 ; i<gx-9; i++)
		{
			for(int l=4;l>=-9;l--)
				colCurrent.push_back( rowI.row(i-l) );
			::cv::minMaxLoc(colCurrent, &minJ, &maxJ, 0, 0, cv::Mat());
			colCurrent = (colCurrent - minJ) / (0.9*(maxJ - minJ)) - 0.1; //%normalization
			//%correlation with kernel
			absColCurrent = cv::sum(abs(colCurrent)).val[0];
			//cv::transpose(colCurrent,Transposed);
			corrFactor = cv::sum(cornerKernelRight *colCurrent.t()).val[0]/(absCornerKernel * absColCurrent);
			if (corrFactor >= cornerMin)
			{
				IRcolumnMax = i;
				break;
			}
			colCurrent.pop_back(colCurrent.rows);
		}

		// Zwracany wynik
		bbox[0] = int(IRcolumnMin);
		bbox[1] = int(IRcolumnMax);
		bbox[2] = int(IRrowMin);
		bbox[3] = int(IRrowMax);

		//
		rowCurrent.release();
		//rowCurrent_transposed.release();
	}

