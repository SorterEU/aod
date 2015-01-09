#ifndef STEM_DESCRIPTORS_H
#define STEM_DESCRIPTORS_H

#include <opencv2/core/core.hpp>

struct MaximumDescriptor {
	size_t i, j, k ;
	float val ;

	MaximumDescriptor(size_t i, size_t j, size_t k, float val)
	{
		this->i = i ; 
		this->j = j ; 
		this->k = k ; 
		this->val = val ;
	}
} ;

struct FeatureDescriptor {
	int decision;
	float response ;
	cv::Mat pos ;
	cv::Mat extent ;

	int Compare(const FeatureDescriptor &fd)
	{
		if (response < fd.response) return -1 ;
		if (response > fd.response) return 1 ;
		return 0 ;
	}	
	
	bool operator == (const FeatureDescriptor &fd) const 
	{
		return response == fd.response ;
	}

	bool operator < (const FeatureDescriptor &fd) const 
	{
		return response < fd.response ;
	}
} ;

#endif
