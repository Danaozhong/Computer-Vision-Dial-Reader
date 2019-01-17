#include "base/gfx/circular_buffer.hpp"


gr_base::circular_buffer::circular_buffer(const cv::Mat &_Image, circular_buffer_orientation _Orientation)
	: m_image(_Image.clone()), m_orientation(_Orientation) {}


cv::Mat gr_base::circular_buffer::operator()(const cv::Rect &roi) const
{
	// check boundaries
	if (gr_base::CIRCULAR_BUFFER_HORIZONTAL == this->m_orientation)
	{
		assert(roi.y >= 0 && roi.y + roi.height <= this->m_image.rows);
	}
	else if (gr_base::CIRCULAR_BUFFER_VERTICAL == this->m_orientation)
	{
		assert(roi.x >= 0 && roi.x + roi.width <= this->m_image.cols);
	}

	// create the buffer
	cv::Mat result(roi.size(), this->m_image.type());

	// check if the image must be unclipped
	cv::Rect roiShifted = roi;
	while (roiShifted.x < 0)
	{
		roiShifted.x += this->m_image.cols;
	}
	while (roiShifted.y < 0)
	{
		roiShifted.y += this->m_image.rows;
	}


	// ultra-lazy implementation...
	for (int x = 0; x != result.cols; ++x)
	{
		for (int y = 0; y != result.rows; ++y)
		{
			size_t srcX = (roiShifted.x + x) % this->m_image.cols;
			size_t srcY = (roiShifted.y + y) % this->m_image.rows;

			if (CV_16U == result.type())
			{
				result.at<unsigned short>(y, x) = this->m_image.at<unsigned short>(srcY, srcX);
			}
			else if (CV_8U == result.type())
			{
				result.at<uchar>(y, x) = this->m_image.at<uchar>(srcY, srcX);
			}
		}
	}
	return result;
}