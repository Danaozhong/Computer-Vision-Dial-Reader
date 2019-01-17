//==============================================================================
//
// Title:       main_algorithms_2d.hpp
// Purpose:     This file contains helper functions for RANSAC algorithms.
//
//==============================================================================

#ifndef _main_algorithms_2d_h_
#define _main_algorithms_2d_h_

#include "common/math/cmn_math_basic.h"
#include "common/math/cmn_math_ellipse.h"
#include <Eigen/StdVector>
#include <opencv2/opencv.hpp>
#include <vector>

namespace grMain
{
	class Line2D
	{
	public:
		Line2D(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2);
		Line2D(const cv::Point2f &p1, const cv::Point2f &p2);
		Eigen::Vector2d p1;
		Eigen::Vector2d p2;
		

		double length() const;
		
		/**
		Returns an normalized vector pointing from p1 to p2.
		*/
		Eigen::Vector2d direction() const;
		
	};

	/**
	Class to represent a list of polygons (as a list of list indices).
	*/
	class IndicesPolygon
	{
	public:
		IndicesPolygon() {};
		IndicesPolygon(const std::vector<unsigned int> &indices);

		std::vector<unsigned int> pointIndices;
	};

	/**
	Comparison operator for polygon indices (it assumes both polygons take the same point list).
	*/
	bool operator== (const IndicesPolygon &polygon1, const IndicesPolygon &polygon2);

	namespace Line2DHelper
	{
		double GetPositionOnLine(const Line2D &line, const Eigen::Vector2d &p);
		double GetSqrDistance(const Line2D &line, const Eigen::Vector2d &p);
		double GetDistance(const Line2D &line, const Eigen::Vector2d &p);

		int ProjectVectorOnLine(const Line2D &line, const Eigen::Vector2d &p, double &return_position, double &return_distance);

	}
	namespace Algorithms2D
	{
		int ScaleToScreenCoordinates(const cv::Point2f &input, cv::Point2f &result, size_t image_width, size_t image_height);
		int ScaleToScreenCoordinates(const Eigen::Vector2d &input, cv::Point2f &result, size_t image_width, size_t image_height);
		int ScaleToScreenCoordinates(const Eigen::Vector2d &input, Eigen::Vector2d &result, size_t image_width, size_t image_height);


		int ScaleToScreenCoordinates(const std::vector<cv::Point2f> &input_points, std::vector<cv::Point2f> &result, size_t image_width, size_t image_height);

		cv::Mat GetContoursUsingCanny(const cv::Mat &input, int threshold);
		cv::Mat GetContoursUsingSobel(const cv::Mat &image_greyscale);
		cv::Mat Dilate(const cv::Mat &mask, int dilation_size);

		cv::Mat CreateMaskFromImage(const cv::Mat &gray_image);

		int GetConvexHull(cv::InputArray points, cv::OutputArray hull);

		int Copy(cv::InputArray source_mat, cv::InputOutputArray dest_mat, cv::Point position, cv::InputArray mask_mat = cv::noArray());
		int FindLinesSuzukiAbe(const cv::Mat &color_image, const cv::Mat &edge_image, std::vector<Line2D> &lines);

		int MergeLines(std::vector<Line2D> &lines, double delta = 10.0, double max_angle_diff_rad = M_PI / 180.0 * 10.0);
		int FindConnectingLines(std::vector<Line2D> &lines, std::vector<Eigen::Vector2d> image_points, std::vector<std::pair<size_t, size_t>> &matches, double max_distance_image_point_line = 7.0, double max_distance_line_line = 10.0, double line_acceptance_angle_rad = M_PI / 180.0 * 10.0, size_t num_of_iterations = 4);
	
		/**
		Experimentary function to find circles. Found as unsuitable to find ellipses but kept for possible future use.
		*/
		int FindCircles(const cv::Mat &input_image);

		class EllipseDetectionAttributes
		{
		public:
			// the minimum number of contour points an ellipse musthave
			size_t minNumOfPoints;

			double minAxisLength;
			double maxAxisLength;
			double maxFittingError;

		};

		int FindEllipses(const cv::Mat &edge_image, const EllipseDetectionAttributes& attributes, std::vector<arMath::Ellipse> &found_ellipses, std::vector<std::vector<Eigen::Vector2d>> &found_contours);
		

		/**
		Extracts the area within an ellipse from one image to a rectangular output buffer.
		The horizontal axis of the output buffer represents the minor axis, the vertical axis the major axis.
		*/
		int ExtractEllipseToImage(const arMath::Ellipse &ellipse, const cv::Mat &source_image, cv::Mat &output_buffer, double rotation_angle = 0.0);

		/**
		Rotates an image around a specific angle.
		*/
		cv::Mat ImageRotate(const cv::Mat &src, const cv::Point &center, double rotation_angle);


		cv::Mat CircularToLinear(const cv::Mat &src, cv::Point const &center, double radius_min, double radius_max, size_t angle_intervals);
		cv::Mat CircularToLinearAverage(const cv::Mat &src, cv::Point const &center, double radius_min, double radius_max);


		void match(cv::Mat const &image, cv::Mat const &templ, cv::Mat1b const &templ_mask, cv::Point const &position, int method, double &corr);

		/**
		Calculates the correlation of two images. All three images must be of the same size, otherwise
		the result will be invalid.
		*/
		double ImageCorrelation(const cv::Mat &img1, const cv::Mat &img2, cv::Mat1b mask = cv::Mat1b());


		template<typename It, typename T>
		typename It FindFirstStep(It const &itr_begin, It const &itr_end, T d_max)
		{
			T last_value = *itr_begin;
			for (auto itr = itr_begin; itr != itr_end; ++itr)
			{
				T current_value = *itr;
				if (std::abs(current_value - last_value) > d_max)
				{
					return itr;
				}
				last_value = current_value;
			}
			return itr_end;
		}


		template<typename It>
		struct gauge_tick_range
		{
			gauge_tick_range(It const &itBegin, It const &itEnd, bool isTickRange)
				: m_isTickRange(isTickRange), m_itBegin(itBegin), m_itEnd(itEnd)
			{}

			template<typename It2>
			gauge_tick_range(It2 const &itBegin, It2 const &itEnd, bool isTickRange)
				: m_isTickRange(isTickRange), m_itBegin(itBegin), m_itEnd(itEnd)
			{}

			bool m_isTickRange;

			It m_itBegin;
			It m_itEnd;


			It begin() const
			{
				return m_itBegin;
			}

			It end() const
			{
				return m_itEnd;
			}
		};

		template<typename It>
		auto find_tick_area(It const &itBegin, It const &itEnd, double TickFrequency) -> std::vector<gauge_tick_range<It>>
		{
			std::vector<gauge_tick_range<It>> gaugeRanges;

			double d_max = 25.0;
			int tick_distance = static_cast<int>(2.0 / TickFrequency);

			// our move iterators
			auto lastStepIt = itBegin;
			auto nextStepIt = itBegin;

			// the range iterators
			auto rangeBeginIt = itBegin;

			bool isCurrentRangeATickRange = false;

			while (true)
			{
				lastStepIt = nextStepIt;
				nextStepIt = FindFirstStep(lastStepIt, itEnd, d_max);

				if (nextStepIt - lastStepIt > tick_distance)
				{
					if (true == isCurrentRangeATickRange)
					{
						// save the current range...
						gaugeRanges.push_back(gauge_tick_range<It>(rangeBeginIt, lastStepIt, true));

						// and the empty range after it.
						gaugeRanges.push_back(gauge_tick_range<It>(lastStepIt, nextStepIt, false));
					}
					else
					{
						// just store an empty range.
						gaugeRanges.push_back(gauge_tick_range<It>(rangeBeginIt, nextStepIt, false));
					}
					
					rangeBeginIt = nextStepIt;
					isCurrentRangeATickRange = false;
				}
				else
				{
					// there are steps within the current range - assume the current range to be a tick area (might need further validation after completion).
					isCurrentRangeATickRange = true;
				}

				if (nextStepIt == itEnd)
				{
					gaugeRanges.push_back(gauge_tick_range<It>(rangeBeginIt, nextStepIt, isCurrentRangeATickRange));
					break;
				}
			}

			return gaugeRanges;
		}

		/**
		Returns an average color value of all color values on an image within a radius at a defined position.
		\param[in] sourceImage The image.
		\param[in] position The center position of the circle of the values to select.
		\param[in] radius The radius of the selection circle.
		\return An average color of all selected pixels.
		*/
		cv::Scalar GetAverageColorValueAtPosition(const cv::Mat &sourceImage, const Eigen::Vector2i &position, int radius);


		bool ArePolygonsEqual(const std::vector<unsigned int> &polygon1, const std::vector<unsigned int> &polygon2);

		
		int FindPolygons(const std::vector<Eigen::Vector2d> &input_points, std::vector<std::pair<unsigned int, unsigned int>> input_edges, std::vector<grMain::IndicesPolygon> &result_polygons);

		CircularMotion PolygonOrientation(const std::vector<Eigen::Vector2d> &input_points, const grMain::IndicesPolygon &input_polygon);

		/**
		Function to shrink a polygon. Each point of the polygon is moved inwards by a user-defineable distance. The direction is determined by the angle of the two adjacent edges.
		The shifting vector for each point is ca    lculated by taking the half of this angle and move the point in this direction.
		*/
		std::vector<Eigen::Vector2d> ShrinkPolygon(const std::vector<Eigen::Vector2d> &polygon, double distance);

		std::vector<Eigen::Vector2d> ScalePolygon(const std::vector<Eigen::Vector2d> &polygon, double scale);

		/**
		Function to check whether a point is within a 2D polygon.
		*/
		bool IsPointInsidePolygon(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector2d &point);
	}
}


#endif