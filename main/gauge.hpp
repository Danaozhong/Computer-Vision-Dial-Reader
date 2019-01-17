/** \file gauge.hpp
\brief Class to represent a detected gauge.

Class to completely describe a gauge on the camera.

Created on:  06/20/2015 at 04:49 PM by Clemens Zangl.
Copyright:   Texelography. All Rights Reserved.
*/

#ifndef _gauge_hpp_
#define _gauge_hpp_

#include <opencv2/opencv.hpp>

namespace gr_main
{
	class gauge
	{
	public:
		/**
		An image of the gauge, used for image vision template matching.
		*/
		cv::Mat m_templateImage;

		/**
		Above image linearized.
		*/
		cv::Mat m_templateImageLinearized;

		/**
		An image of the dial pointer.
		*/
		cv::Mat m_templatePointerLinearized;

		/*
		Binary mask used to mask out the pointer of the gauge.
		*/
		cv::Mat m_templatePointerMask;

		double m_scaleBeginAngle;
		double m_scaleEndAngle;

		size_t m_scaleBeginLinearized;
		size_t m_scaleEndLinearized;

		double m_tickFrequency;

		double m_minScaleValue;
		double m_maxScaleValue;

		double m_pointerAngle;
		size_t m_pointerLinearized;



	};

	int find_gauge_pointer(gauge &Gauge);
}


#endif