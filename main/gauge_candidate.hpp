/** \file gauge_candidate.hpp
\brief Class to represent an image area that possibily represents a gauge.

Detail description

Created on:  06/20/2015 at 04:49 PM by Clemens Zangl.
Copyright:   Texelography. All Rights Reserved.
*/

#ifndef _gauge_candidate_hpp_
#define _gauge_candidate_hpp_

#include "cmn_math_ellipse.h"
#include <opencv2/opencv.hpp>

namespace gr_main
{
	class gauge_candidate
	{
	public:	
		/**
		The ellipse on the current camera image. 
		*/
		arMath::Ellipse m_ellipse;

		cv::Mat image_area;

		/**
		The angle, how much the ellipse coordinate system must be rotated to match the gauge.
		*/
		double m_rotationAngle;
		/**
		The version of the gauge on the current provided camera image.
		*/
		
		double correlation;
	};
}


#endif