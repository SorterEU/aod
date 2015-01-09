// Plik rod_Ellipse.cpp

	//
	// Funkcja pomocnicza 2: ErosionEllipse()
	//

	/// Global variables
	::cv::Mat src, erosion_dst, dilation_dst;
	
	int erosion_size = 1;
	int dilation_elem = 1;
	int dilation_size = 1;
	int const max_elem = 2;
	int const max_kernel_size = 21;

	void ErosionEllipse( ::cv::Mat &src, ::cv::Mat &dst)
	{
		int erosion_type = ::cv::MORPH_ELLIPSE;
		::cv::Mat element = ::cv::getStructuringElement( erosion_type,
                       ::cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                       ::cv::Point( erosion_size, erosion_size ) );
		/// Apply the erosion operation
		::cv::erode( src, dst, element );
		element.release();
	}

	//
	//
	// Funkcja pomocnicza 3: DilationEllipse()
	//

	void DilationEllipse( ::cv::Mat &src, ::cv::Mat &dst)
	{
		int dilation_type = ::cv::MORPH_ELLIPSE;
		::cv::Mat element = ::cv::getStructuringElement( dilation_type,
                       ::cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                       ::cv::Point( dilation_size, dilation_size ) 
					   );
		/// Apply the dilation operation
		::cv::dilate( src, dst, element );
		element.release();
  
	}
