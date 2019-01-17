/** \file main.cpp
\brief Class to read a gauge manufactured by HELIOS via the COM interface.


Created on:  05/28/2015 at 06:04 PM by Clemens Zangl.
Copyright:   Texelography. All Rights Reserved.
*/

#include "base/gfx/circular_buffer.hpp"
#include "main/2d/main_algorithms_2d.hpp"
#include "common/math/cmn_math_advanced.hpp"
#include "cmn_math_matrices.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/density.hpp>
#include "main/gauge_candidate.hpp"
#include "main/gauge.hpp"


auto calculate_double_line_array(size_t column, const cv::Mat &linear_color, int _RowBegin = -1, int _RowEnd = -1) -> std::vector<double>
{

	std::vector<double> resultVector;

	if (-1 == _RowEnd)
	{
		_RowEnd = linear_color.rows;
	}
	if (-1 == _RowBegin)
	{
		_RowBegin = 0;
	}

	resultVector.reserve(linear_color.rows);
	for (size_t i = _RowBegin; i != _RowEnd ; ++i)
	{
		resultVector.push_back(static_cast<double>(linear_color.at<uchar>(i, column)));
	}
	return resultVector;
};


cv::Mat1b CreateCircularMask(cv::Size const & size, cv::Point center, double min_radius, double max_radius, double begin_angle, double end_angle)
{
	cv::Mat1b mask = cv::Mat1b::zeros(size);

	auto IsPointInCircleArea = [&](const cv::Point &current_point) -> bool
	{
		// check if the distance to the center point is smaller than the radius
		auto centerVector = current_point - center;
		double distance = cv::norm(centerVector);
		if (max_radius >= distance && distance >= min_radius)
		{
			double angle = arMath::CompleteAngle2D(Eigen::Vector2d(0.0, 1.0), ToEVector2d(centerVector));
			if (angle >= begin_angle && angle <= end_angle)
			{
				return true;
			}
		}
		return false;
	};

	for (size_t x = 0; x != mask.cols; ++x)
	{
		for (size_t y = 0; y != mask.rows; ++y)
		{
			if (IsPointInCircleArea(cv::Point(x, y)))
			{
				mask.at<uchar>(x, y) = 255;
			}
		}
	}
	return mask;
}

int create_triangle_mask(cv::Mat1b &mask, cv::Point _TriangleBegin, double _TriangleRotationAngle, double _TriangleHeight, double _TriangleWidth)
{
	double sinInv = sin(-_TriangleRotationAngle);
	double cosInv = cos(-_TriangleRotationAngle);
	_TriangleWidth = _TriangleWidth / 2.0;
	auto is_point_in_triangle = [&](cv::Point const &point) -> bool
	{
		// first, rotate the point onto the triangle
		cv::Point2d pointd = static_cast<cv::Point2d>(point - _TriangleBegin);
		cv::Point2d rotatedPoint{ cosInv * pointd.x - sinInv * pointd.y, cosInv * pointd.y + sinInv * pointd.x };

		// do a rectangular check.
		if (rotatedPoint.x < 0.0 || rotatedPoint.x > _TriangleHeight || std::abs(rotatedPoint.y) > _TriangleWidth)
		{
			return false;
		}

		// the point definitely lies inside of the outer rectangular hull of the triangle. Check if it also lies inside of the triangle.
		double currentTriangleWidth = (1.0 - rotatedPoint.x / _TriangleHeight) * _TriangleWidth;
		if (std::abs(rotatedPoint.y) > currentTriangleWidth)
		{
			return false;
		}
		return true;
	};

	for (size_t x = 0; x != mask.cols; ++x)
	{
		for (size_t y = 0; y != mask.rows; ++y)
		{
			if (is_point_in_triangle(cv::Point(x, y)))
			{
				mask.at<uchar>(y, x) = 255;
			}
		}
	}
	return 0;
}

int find_ticks(gr_main::gauge_candidate &_GaugeCandidate, double &_Ticks, int _RowBegin = -1, int _RowEnd = -1)
{

	// convert the radial image to a linear image to identify the different gauge areas (e.g. the ticks or the numbers).
	cv::Point gaugeCenter{ _GaugeCandidate.image_area.cols / 2, _GaugeCandidate.image_area.rows / 2 };
	double radius{ static_cast<double>(_GaugeCandidate.image_area.cols) / 2.0 };

	cv::Mat linear_color = grMain::Algorithms2D::CircularToLinear(_GaugeCandidate.image_area, gaugeCenter, 0, radius, 720);

	//cv::imshow("Original Gauge", _GaugeCandidate.image_area);
	//cv::imshow("Linearized", linear_color);




	size_t tickScanColBegin = static_cast<size_t>(static_cast<double>(linear_color.cols) * 0.75);
	size_t tickScanColEnd = linear_color.cols;

	double minFrequency = 0.005;
	double maxFrequency = 0.4;


	using frequency_accumulator = boost::accumulators::accumulator_set<double, boost::accumulators::features<boost::accumulators::tag::count, boost::accumulators::tag::density>>;
	frequency_accumulator frequencyAccumulator(boost::accumulators::tag::density::num_bins = 50, boost::accumulators::tag::density::cache_size = 10);

	// search for the frequencies....
	const size_t N = 7;				// a number larger than the number of frequencies searched.
	const double threshold = 3.0;	// the threshold used to separate noise from signal space.

	for (size_t i = tickScanColBegin; i != tickScanColEnd; ++i)
	{
		// copy the current colums and perfom a spectral analysis
		auto frequencies = Math::spectral_estimation_MUSIC(calculate_double_line_array(i, linear_color, _RowBegin, _RowEnd), N, threshold, 1.0);

		// copy the found frequencies into the accumulator
		for (auto &frequency : frequencies)
		{
			// filter min and max frequency
			if (frequency.first < minFrequency) // || frequency.first > maxFrequency)
			{
				continue;
			}
			//...and add to accumulator!
			frequencyAccumulator(frequency.first);
		}
	}



	// identify frequencies using a histogram to find the most common frequencies
	auto histogram = boost::accumulators::density(frequencyAccumulator);

	size_t minNumberOfItems = 3;
	size_t numOfElements = boost::accumulators::count(frequencyAccumulator);

	std::vector<double> identifiedFrequencies;

	for (auto binItr = histogram.begin(); binItr != histogram.end(); ++binItr)
	{
		if (static_cast<size_t>(static_cast<double>(numOfElements)* binItr->second) >= minNumberOfItems)
		{
			identifiedFrequencies.push_back(binItr->first);
		}
	}

	if (identifiedFrequencies.size() > 0)
	{
		_Ticks = identifiedFrequencies.front();
		return 0;
	}
	return -1;
}

int find_needle(gr_main::gauge &_Gauge)
{
	// get a template of a few ticks
	const double numOfTicksForTemplate = 2.0;
	const double numOfTicksStep = 0.5;


	const size_t templateHeight = static_cast<size_t>(numOfTicksForTemplate / _Gauge.m_tickFrequency);
	const size_t compareHeight = static_cast<size_t>(numOfTicksStep / _Gauge.m_tickFrequency);


	//cv::Mat dialTemplate = _Gauge.m_templateImageLinearized(cv::Rect(0, _Gauge.m_scaleBeginLinearized + 10, _Gauge.m_templateImageLinearized.cols, templateHeight));



	const size_t numOfPixels = _Gauge.m_scaleEndLinearized - _Gauge.m_scaleBeginLinearized;

	size_t numOfSteps = (numOfPixels + templateHeight) / compareHeight;

	// calculate the correlation values for the dial and comparison
	std::vector<double> corrValues;
	std::vector<cv::Mat> corrImages;
	size_t bestIndex = 0;
	size_t worstIndex = 0;


	// create a triangle template
	cv::Mat1b dialTemplateMask = cv::Mat1b::zeros(cv::Size(_Gauge.m_templateImageLinearized.cols, templateHeight));
	create_triangle_mask(dialTemplateMask, cv::Point{ 0, static_cast<int>(templateHeight) / 2 }, 0.0, static_cast<double>(_Gauge.m_templateImageLinearized.cols), static_cast<double>(templateHeight));
	
	cv::Mat dialTemplate = cv::Mat(dialTemplateMask.size(), _Gauge.m_templateImageLinearized.type(), cv::Scalar(0));

	gr_base::circular_buffer dialBuffer(_Gauge.m_templateImageLinearized, gr_base::CIRCULAR_BUFFER_BOTH);

	for (size_t i = 0; i != numOfSteps; ++i)
	{
		int positionY = _Gauge.m_scaleBeginLinearized - templateHeight + i * compareHeight; //static_cast<size_t>(static_cast<double>(i) * numOfTicksStep / _Gauge.m_tickFrequency);

		//assert(positionY <= static_cast<int>(_Gauge.m_scaleEndLinearized));

		// copy rect
		cv::Mat currentCompareArea = dialBuffer(cv::Rect(0, positionY, _Gauge.m_templateImageLinearized.cols, templateHeight));
		corrImages.push_back(currentCompareArea);

		const double corrValue = grMain::Algorithms2D::ImageCorrelation(currentCompareArea, dialTemplate);
		corrValues.push_back(corrValue);

		if (corrValue < corrValues[bestIndex])
		{
			bestIndex = i;
		}
		if (corrValue > corrValues[worstIndex])
		{
			worstIndex = i;
		}

		std::cout << "current Corr: " << corrValue << std::endl;
	}

	_Gauge.m_pointerLinearized = _Gauge.m_scaleBeginLinearized - templateHeight / 2 + bestIndex * compareHeight;
	_Gauge.m_pointerAngle = _Gauge.m_pointerLinearized / _Gauge.m_templateImageLinearized.rows * 2.0 * M_PI;
	// show the results:
	cv::imshow("Template", dialTemplate);
	cv::imshow("Best", corrImages[bestIndex]);
	cv::imshow("Worst", corrImages[worstIndex]);

	cv::imshow("first", corrImages.front());
	cv::imshow("last", corrImages.back());

	
	cv::Mat dialExtended = dialBuffer(cv::Rect(-100, -100, 800, 800));

	cv::imshow("Extended", dialExtended);
	
	return 0;

}


int calculate_symmetry(const cv::Mat &_GrayscaleImage, double &_Corr)
{
	double centerX = static_cast<double>(_GrayscaleImage.cols) / 2.0;
	double centerY = static_cast<double>(_GrayscaleImage.rows) / 2.0;
	cv::Point center(static_cast<int>(centerX), static_cast<int>(centerY));
	double radius = centerX;

	return 0;
}

std::vector<uchar> calculateRadialHistogram(const cv::Mat &grayscale_image, double &corr)
{
	std::vector<uchar> colorValues;

	
	

	double centerX = static_cast<double>(grayscale_image.cols) / 2.0;
	double centerY = static_cast<double>(grayscale_image.rows) / 2.0;
	double radius = centerX;
	cv::Point center(static_cast<int>(centerX), static_cast<int>(centerY));



	// create the template. Cut out an area of the source image.
	double templPieSize = 360.0 / 180.0 * M_PI;
	double templBeginAngle = 0.0;
	// create a mask
	cv::Mat templMask = CreateCircularMask(grayscale_image.size(), center, 0.0, radius, templBeginAngle, templBeginAngle + templPieSize);
	// and use this mask to cut-out an area of the source image.
	cv::Mat templ = cv::Mat::zeros(grayscale_image.size(), grayscale_image.type());

	grayscale_image.copyTo(templ, templMask);

	
	templ = grMain::Algorithms2D::GetContoursUsingSobel(templ);
	//auto compareImage = grMain::Algorithms2D::GetContoursUsingSobel(grayscale_image);

	cv::imshow("template", templ);
	cv::imshow("gray", grayscale_image);

	size_t numOfValues = 10;
	double angleStep = 1.0 / 180.0 * M_PI / numOfValues;
	colorValues.reserve(numOfValues);
	

	auto compare_template_area = [&templ, &templMask, &center, &centerX, &centerY, &radius](double _AngleBegin, double _AngleStep, size_t _NumOfValues) -> double
	{
		double _Corr = 0.0;

		// rotate the template and calculate the 
		for (size_t i = 0; i != _NumOfValues; i++)
		{
			double angle = _AngleBegin + static_cast<double>(i)* _AngleStep;
			int x = static_cast<int>(centerX + cos(angle) * radius);
			int y = static_cast<int>(centerY + sin(angle) * radius);

			auto current_templ = grMain::Algorithms2D::ImageRotate(templ, center, angle);
			auto current_templ_mask = grMain::Algorithms2D::ImageRotate(templMask, center, angle);

			double currentCorr = 0.0;
			grMain::Algorithms2D::match(templ, current_templ, current_templ_mask, cv::Point(0, 0), cv::TM_CCORR, currentCorr);
			if (currentCorr > _Corr)
			{
				_Corr = currentCorr;
			}
		}
		return _Corr;
	};

	corr = 0.0;
	corr = compare_template_area(60.0 / 180.0 * M_PI, angleStep, numOfValues);
	corr += compare_template_area(270.0 / 180.0 * M_PI, angleStep, numOfValues);
	return colorValues;
}


int findGauges(const cv::Mat &color_image_input)
{
	cv::Mat imgGrayScale;
	cv::cvtColor(color_image_input, imgGrayScale, cv::COLOR_BGR2GRAY);

	auto edgeImage = grMain::Algorithms2D::GetContoursUsingCanny(imgGrayScale, 30);


	grMain::Algorithms2D::EllipseDetectionAttributes attributes;
	attributes.maxAxisLength = 500;
	attributes.maxFittingError = 10000;
	attributes.minAxisLength = 20;
	attributes.minNumOfPoints = 100;
	std::vector<arMath::Ellipse> foundEllipses;
	std::vector<std::vector<Eigen::Vector2d>> foundContours;
	grMain::Algorithms2D::FindEllipses(edgeImage, attributes, foundEllipses, foundContours);

	cv::imshow("Edges", edgeImage);

	cv::Mat gaugeImage;
	gaugeImage = color_image_input.clone();
	cv::RNG rng(123);




	std::vector<gr_main::gauge_candidate> candidates;

	for (auto itr = foundEllipses.begin(); itr != foundEllipses.end(); itr++)
	//for (size_t i = 6; i != 8; i++)
	{
		// copy the ellipse into a new buffer
		cv::Mat outputBuffer = cv::Mat::ones(cv::Size{ 300, 300 }, imgGrayScale.type());

		arMath::EllipseHelper::Draw(gaugeImage, *itr, cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 2);

		grMain::Algorithms2D::ExtractEllipseToImage(*itr, imgGrayScale, outputBuffer);

		

		double currentCorrelation{ 0.0 };
		calculateRadialHistogram(outputBuffer, currentCorrelation);
		gr_main::gauge_candidate candidate{ *itr, outputBuffer, 0.0, currentCorrelation };
		candidates.push_back(candidate);
		// radially check the output buffer for similarities
		//std::vector<uchar> grayscaleValues;

	}

	

	// sort the candidate list
	std::sort(candidates.begin(), candidates.end(), [](gr_main::gauge_candidate const &c1, gr_main::gauge_candidate const &c2)
	{
		return (c1.correlation < c2.correlation);
	});

	std::reverse(candidates.begin(), candidates.end());
	for (auto itr = candidates.begin(); itr != candidates.end(); itr++)
	{
		cv::imshow("Candidate " + std::to_string(itr - candidates.begin()), itr->image_area);
		std::cout << "Candidate " << (itr - candidates.begin()) << ": " << itr->correlation << std::endl;
	}

	auto gaugeItr = candidates.end();

	double freq = 0.0;
	// evaluate the candidates depending on the probability
	for (auto itr = candidates.begin(); itr != candidates.end(); ++itr)
	{
		if (0 == find_ticks(*itr, freq))
		{
			gaugeItr = itr;
			break;
		}
	}
	
	if (candidates.end() == gaugeItr)
	{
		// no gauge found!
		return -1;
	}

	// select the first candidate.
	auto gauge = *gaugeItr;
	cv::Point gaugeCenter{ gauge.image_area.cols / 2, gauge.image_area.rows / 2 };
	double radius{ static_cast<double>(gauge.image_area.cols) / 2.0 };

	cv::Mat linear_color = grMain::Algorithms2D::CircularToLinear(gauge.image_area, gaugeCenter, 0, radius, 720);

	
	// save the 73 col
	size_t col = 133;
	/*
	std::ofstream file_stream;
	cv::Mat line(cv::Size(1, linear_color.rows), linear_color.type());

		
	file_stream.open("data.txt");
	for (size_t i = 0; i != linear_color.rows; ++i)
	{
		line.at<uchar>(i, 0) = linear_color.at<uchar>(i, col);
		file_stream << static_cast<int>(linear_color.at<uchar>(i, col)) << " ";
	}


	file_stream.close();
	*/
	std::vector<double> line_double = calculate_double_line_array(col, linear_color);


	


	//auto frequencies2 = Math::spectral_estimation_MUSIC(calculate_double_line_array(col + 1), 8, 3.0, 1.0);
	// find gauge if area 
	//cv::imshow("Line", line);
	
	// find the empty area


	//double freq = 0.1839;
	
	auto TickAreaList = grMain::Algorithms2D::find_tick_area(line_double.begin(), line_double.end(), freq);



	auto calculate_angle = [&line_double](size_t index) -> double
	{
		return static_cast<double>(index) / static_cast<double>(line_double.size()) * 2.0 *M_PI;
	};

	// check if the tickAreaList
	if (TickAreaList.front().m_isTickRange && TickAreaList.back().m_isTickRange)
	{
		// rotate the image in such a way that start point equals to the lowest point of the gauge scale.
		gauge.m_rotationAngle = calculate_angle(TickAreaList.back().begin() - line_double.begin());

		gauge.image_area = cv::Mat::ones(cv::Size{ 300, 300 }, imgGrayScale.type());
		grMain::Algorithms2D::ExtractEllipseToImage(gauge.m_ellipse, imgGrayScale, gauge.image_area, gauge.m_rotationAngle);
	}
	
	linear_color = grMain::Algorithms2D::CircularToLinear(gauge.image_area, gaugeCenter, 0, radius, 720);

	line_double = calculate_double_line_array(col, linear_color);
	TickAreaList = grMain::Algorithms2D::find_tick_area(line_double.begin(), line_double.end(), freq);

	// recalculate the tick frequency, based on the newly found tick area boundaries



	// draw a few test lines
	cv::Mat tickImage;
	cv::cvtColor(gauge.image_area, tickImage, cv::COLOR_GRAY2BGR); // .clone();



	auto draw_tick = [&tickImage, &gaugeCenter](double Angle, bool _IsMajorTick, const cv::Scalar &_Color)
	{
		if (_IsMajorTick)
		{
			cv::line(tickImage, gaugeCenter, gaugeCenter + cv::Point(static_cast<int>(cos(Angle) * 150.0), static_cast<int>(sin(Angle) * 150.0)), _Color);
		}
		else
		{
			cv::line(tickImage, gaugeCenter + cv::Point(static_cast<int>(cos(Angle) * 100.0), static_cast<int>(sin(Angle) * 100.0)),
								gaugeCenter + cv::Point(static_cast<int>(cos(Angle) * 150.0), static_cast<int>(sin(Angle) * 150.0)), _Color);
		}
	};

	auto draw_tick_itr = [&tickImage, &gaugeCenter, &line_double, &draw_tick, &calculate_angle](std::vector<double>::const_iterator &It, bool IsMajorTick)
	{
		double Angle = calculate_angle(It - line_double.begin());
		draw_tick(Angle, IsMajorTick, cv::Scalar(0, 100, 0));
	};

	for (auto tickItr = TickAreaList.begin(); tickItr != TickAreaList.end(); ++tickItr)
	{
		// draw the begin and end tick
		draw_tick_itr(tickItr->begin(), true);
		draw_tick_itr(tickItr->end(), true);

		if (false == tickItr->m_isTickRange)
		{
			continue;
		}

		// draw the ticks in between
		size_t numOfIterators = tickItr->end() - tickItr->begin();
		double totalAngle = calculate_angle(numOfIterators);
		size_t numOfTicks = static_cast<size_t>(static_cast<double>(numOfIterators) * freq);
		double angle_step = totalAngle / static_cast<double>(numOfTicks);
		double BeginAngle = calculate_angle(tickItr->begin() - line_double.begin());

		for (size_t i = 0; i != numOfTicks; ++i)
		{
			double current_angle = BeginAngle + angle_step * static_cast<double>(i);
			draw_tick(current_angle, false, cv::Scalar(0, 0, 0));
		}
	}

	gr_main::gauge FinalGauge;

	FinalGauge.m_scaleBeginLinearized = TickAreaList.front().begin() - line_double.begin();
	FinalGauge.m_scaleEndLinearized = TickAreaList.front().end() - line_double.begin();

	FinalGauge.m_scaleBeginAngle = calculate_angle(FinalGauge.m_scaleBeginLinearized);
	FinalGauge.m_scaleEndAngle = calculate_angle(FinalGauge.m_scaleEndLinearized);

	FinalGauge.m_templateImage = gauge.image_area.clone();
	FinalGauge.m_templateImageLinearized = linear_color.clone();
	FinalGauge.m_tickFrequency = freq;
	find_needle(FinalGauge);

	draw_tick(FinalGauge.m_pointerAngle, true, cv::Scalar(0, 255, 0));


	cv::imshow("Ticks", tickImage);
	cv::imshow("Linearlized final", linear_color);
	return 0;
}

int main(int argc, const char* argv[])
{
	// the raw color image
	cv::Mat gaugeImage = cv::imread("data\\gauge.png");

	findGauges(gaugeImage);
	
	Math::TestMath::TestXCorr();
	math::matrix_tester::test_diag();
	cv::imshow("Gauge", gaugeImage);
	cv::waitKey();
}
