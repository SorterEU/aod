// Plik rod_DetectROI.cpp


	//
	// Funkcja pomocnicza 4: DetectROI()
	//

	void DetectROI(::cv::Mat &src, int bbox[])
	{
		// Wewnętrzne parametry:
		float GRADIENT_THRESH = 100.0; // minimalny gradient dla 1-wym. rozkładu po kolumnach lub wierszach
		float MIN_VALUE_RATIO = 0.15; // względem minimum jasności obrazu w centralnym obszarze
		int MIN_BORDER = 20; // minimalny brzeg obrazu, wyrażony w pikselach

		int columnMin, columnMax, rowMin, rowMax;
	    int sy = src.rows;
	    int sx = src.cols;
	    
		int Cx = (int)(((double)sx) * 0.5); // położenie środka obrazu
		int Cy = (int)(((double)sy) * 0.5);
		int Cx1 = (int)(((double)sx) * 0.3333); // 1/3 długości wiersza
		int Cx2 = (int)(((double)sx) * 0.6666); // 2/3 długości wiersza
		int Cy1 = (int)(((double)sy) * 0.3333); // 1/3 długości kolumny
		int Cy2 = (int)(((double)sy) * 0.6666); // 2/3 długości kolumny
		
	    int sum1, sum2;
	    ::cv::Mat columnJ = ::cv::Mat::zeros(sx, 1, CV_32SC1);
	    ::cv::Mat rowI = ::cv::Mat::zeros(sy, 1, CV_32SC1);
	    ::cv::Mat columnJGrad, rowIGrad;
	    rowI.copyTo(rowIGrad);
	    columnJ.copyTo(columnJGrad);
	    columnJGrad.convertTo(columnJGrad,CV_32FC1);
	    rowIGrad.convertTo(rowIGrad,CV_32FC1);
	    double maxColumnVal, maxRowVal, minColumnVal, minRowVal;
		int threshMax;
	    int k;


	    //% Funkcja obrazu sumowana po kolumnach lub wierszach
            //%     projections to rows and columns
//std::cout<< "ColumnJ: ";
	    for (int j=0;j<sx;j++) {
	    	columnJ.at<int>(j,0)=::cv::sum(src.col(j)).val[0];
//std::cout<< columnJ.at<int>(j,0)<<";" ;

		}
//std::cout << std::endl;
//std::cout<< "rowI: ";
	    for (int i=0;i<sy;i++) {
	    	rowI.at<int>(i,0)=::cv::sum(src.row(i)).val[0];
//std::cout<< rowI.at<int>(i,0)<<";";  
		}
//std::cout << std::endl;

	    //% gradienty obu funkcji sumarycznych obrazu
	    //% gradients of column and row distributions
	    for (int j=10-1;j<sx-9;j++)
	      {
	    	sum1=0; sum2=0;
	    	for (int k=1;k<=9;k++)//columnJGrad(j) = (sum(columnJ(j+1:j+9)) - sum(columnJ(j-9:j-1))) * 0.05;
	        {
	          sum1+=columnJ.at<int>(j+k,0);//::cv::sum(columnJ.row(j+k));
	          sum2+=columnJ.at<int>(j-k,0);//::cv::sum(columnJ.row(j-k));//(10-k)
	        }
	        columnJGrad.at<float>(j,0)= (sum1 - sum2)*0.05;
	      }
	    for (int i=10-1;i<sy-9;i++)
	      {
	    	sum1=0; sum2=0;
	        for (int k=1;k<=9;k++)//rowIGrad(i) = (sum(rowI(i+1:i+9)) - sum(rowI(i-9:i-1))) * 0.05;
	        {
	        	sum1+=rowI.at<int>(i+k,0);//::cv::sum(rowI.row(i+k));
	        	sum2+=rowI.at<int>(i-k,0);//::cv::sum(rowI.row(i-k));//(10-k)
	        }
	        rowIGrad.at<float>(i,0)= (sum1 - sum2)*0.05;
	      }
		
		// Wyznaczenie prostokątnego ROI
		// Przytnij kolumny do środkowego obszaru obrazu
		::cv::Mat columnJCent = columnJ(::cv::Rect(0, Cx1, 1, Cx2 - Cx1 + 1)).clone();
		// Wyznacz wartość minimalną w środkowym obszarze
	    ::cv::minMaxLoc(columnJCent, &minColumnVal, &maxColumnVal, 0, 0, ::cv::Mat());
		// Wyznacz spodziewaną wartość tła
		float sumb = 0.0;
		for (int i=0; i<3; i++) {
			sumb += columnJ.at<int>(i,0);
			sumb += columnJ.at<int>(sx-i-1,0);
		}
		sumb = sumb / 6.0;
		if (sumb < minColumnVal)
			threshMax = int (sumb + (minColumnVal - sumb) * MIN_VALUE_RATIO);
		else
			threshMax = int (minColumnVal);
		
std::cout<<"threshMax column:"<< threshMax<< std::endl;

	    k = MIN_BORDER;
		int curValue;
		int prevValue = columnJ.at<int>(Cx1,0);
	    for (int j=Cx1 - 1; j>= MIN_BORDER; j--) //for j= round(sx/4.0): -1: 2
		{
			curValue = columnJ.at<int>(j,0);
			if ((curValue < prevValue) && (prevValue < threshMax))
	        {
	          k = j;
	          break;
	        }
			prevValue = curValue;
		}

	    columnMin = MIN_BORDER;
		float curGradValue;
		float prevGradValue = columnJGrad.at<float>(k,0);
		if (k > MIN_BORDER) {
	    for (int i=k-1; i>= MIN_BORDER; i--) //for i = k: -1: 1
		{
			curGradValue = columnJGrad.at<float>(i,0);
			if ((curGradValue < prevGradValue) && (curGradValue >= GRADIENT_THRESH))
	        {
	          columnMin = i;
	          break;
	        }
			prevGradValue = curGradValue;
		}
		}

	    k = sx - MIN_BORDER - 1;
		int maxind = k;
		prevValue = columnJ.at<int>(Cx2,0);
	    for (int j=Cx2 + 1; j<= maxind ; j++) //for j= round(sx * 3.0/4.0): 1: sx
		{
			curValue = columnJ.at<int>(j,0);
			if ((curValue < prevValue) && (prevValue < threshMax))
	        {
	          k = j;
	          break;
	        }
			prevValue = curValue;
		}

	    columnMax = maxind;
		if (k < maxind) {
		prevGradValue = columnJGrad.at<float>(k,0);
	    for (int i=k+1; i<= maxind; i++) //for i = k: 1: sx - MIN_BORDER
		{
			curGradValue = columnJGrad.at<float>(i,0);
			if ((curGradValue > prevGradValue) && (curGradValue <= - GRADIENT_THRESH))
	        {
	          columnMax = i;
	          break;
	        }
			prevGradValue = curGradValue;
		}
		}

	    // Row search
		::cv::Mat rowICent = rowI(::cv::Rect(0, Cy1, 1, Cy2 - Cy1 + 1)).clone();
	    ::cv::minMaxLoc(rowICent, &minRowVal, &maxRowVal, 0, 0, ::cv::Mat());
		//float 
		sumb = 0.0;
		for (int i=0; i<3; i++) {
			sumb += rowI.at<int>(i,0);
			//sumb += rowI.at<int>(sy-i-1,0);
		}
		sumb = sumb / 3.0;
		if (sumb < minRowVal)
			threshMax = int (sumb + (minRowVal - sumb) * MIN_VALUE_RATIO);
		else
			threshMax = int (minRowVal);

std::cout<<"threshMax row:"<< threshMax<< std::endl;
	    k = MIN_BORDER;
		prevValue = rowI.at<int>(Cy1,0);
	    for (int j=Cy1 - 1; j>= MIN_BORDER ; j--)//for j= round(sy/4.0): -1: 2
		{
			curValue = rowI.at<int>(j,0); 
			if ((curValue < prevValue) && (curValue < threshMax))
	        {
	          k = j;
	          break;
	        }
			prevValue = curValue;
		}
	    //rowIGrad
	    rowMin = MIN_BORDER;
		if (k > MIN_BORDER) {
		prevGradValue = rowIGrad.at<float>(k,0);
	    for (int i=k-1; i>= MIN_BORDER; i--)//for i = k: -1: 1
		{
			curGradValue = rowIGrad.at<float>(i,0);
	        if ((curGradValue < prevGradValue) && (curGradValue >= GRADIENT_THRESH))
			{
	            rowMin = i;
	            break;
	        }
			prevGradValue = curGradValue;
		}
		}

	    k = sy - MIN_BORDER - 1;
		maxind = k;
		prevValue = rowI.at<int>(Cy2,0);
	    for (int j=Cy2 + 1; j>=maxind; j++)//for j= round(sy * 3.0/4.0): 1: sy
		{
			curValue = rowI.at<int>(j,0);
			if ((curValue < prevValue) && (curValue < threshMax))
	        {
	          k = j;
	          break;
	        }
			prevValue = curValue;
		}

	    rowMax = maxind;
		if (k <maxind) {
	    prevGradValue = rowIGrad.at<float>(k,0);
		for (int i=k+1; i>=maxind; i++)//for i = k: 1: sy
		{
			curGradValue = rowIGrad.at<float>(i,0);
			if ((curGradValue > prevGradValue) && (curGradValue <= - GRADIENT_THRESH))
	        {
	          rowMax = i;
	          break;
	        }
			prevGradValue = curGradValue;
		}
		}

/*std::cout << std::endl;
std::cout<< "rowI: ";
	    for (int ii=0; ii<sy; ii++) {
std::cout<< rowIGrad.at<float>(ii,0)<<";";  
		}
std::cout << std::endl;
*/
	    // rozszerz  ramkę
        //
	    columnMin = columnMin - 2;
	    if (columnMin < MIN_BORDER) {
			columnMin = MIN_BORDER;
		}
		else
			columnMin = (int)((columnMin + MIN_BORDER) * 0.5);

	    columnMax = columnMax + 2;
	    if (columnMax > (sx - MIN_BORDER)) 	{
			columnMax = sx- MIN_BORDER;
		}
		else
			columnMax = (int)((columnMax + sx - MIN_BORDER) * 0.5);

	    rowMin = rowMin - 2;
	    if (rowMin <MIN_BORDER) {
			rowMin = MIN_BORDER;
		}
		else
			rowMin = (int)((rowMin + MIN_BORDER) * 0.5);

	    rowMax = rowMax + 2;
	    if (rowMax > (sy - MIN_BORDER)) {
			rowMax = sy- MIN_BORDER;
		}
		else 
			rowMax = (int)((rowMax + sy - MIN_BORDER) * 0.5);

		// Zwracany wynik
		bbox[0] = columnMin;
		bbox[1] = columnMax;
		bbox[2] = rowMin;
		bbox[3] = rowMax;

		//
	    columnJ.release();
	    rowI.release();
	    columnJGrad.release();
	    rowIGrad.release();

	}
