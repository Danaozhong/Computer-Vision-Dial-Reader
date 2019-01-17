/** \file circular_buffer.hpp
\brief 

Class to completely describe a gauge on the camera.

Created on:  06/23/2015 at 01:06 PM by Clemens Zangl.
Copyright:   Texelography. All Rights Reserved.
*/

#ifndef _circular_buffer_hpp_
#define _circular_buffer_hpp_

#include <opencv2/opencv.hpp>

namespace gr_base
{
	enum circular_buffer_orientation
	{
		CIRCULAR_BUFFER_VERTICAL,
		CIRCULAR_BUFFER_HORIZONTAL,
		CIRCULAR_BUFFER_BOTH
	};

	class circular_buffer
	{
	private:
		/**
		The cv::Mat image, which is used
		*/
		cv::Mat m_image;

		circular_buffer_orientation m_orientation;

	public:
		/**
		Constructor.
		*/
		circular_buffer(const cv::Mat &_Image, circular_buffer_orientation _Orientation);

		/**
		Function to extract a subarea of an circular image buffer.
		*/
		cv::Mat operator()(const cv::Rect &roi) const;
	};

	
}


#endif