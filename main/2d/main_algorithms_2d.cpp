//==============================================================================
//
// Title:       main_algorithms.cpp
// Purpose:     See header!
//==============================================================================

#include "common/math/cmn_math_basic.h"
#include "common\math\cmn_math_advanced.hpp"

#include "main\2d\main_algorithms_2d.hpp"
#include <Eigen\StdVector>
#include <numeric>


namespace grMain
{

	Line2D::Line2D(const cv::Point2f &p1, const cv::Point2f &p2)
	{
		this->p1 = ToEVector2d(p1);
		this->p2 = ToEVector2d(p2);
	}

	double Line2D::length() const
	{
		return (this->p2 - this->p1).norm();
	}

	Eigen::Vector2d Line2D::direction() const
	{
		return (this->p2 - this->p1).normalized();
	}

	IndicesPolygon::IndicesPolygon(const std::vector<unsigned int> &indices)
		: pointIndices(indices)
	{}

	bool operator== (const IndicesPolygon &polygon1, const IndicesPolygon &polygon2)
	{
		return Algorithms2D::ArePolygonsEqual(polygon1.pointIndices, polygon2.pointIndices);
	}

	double Line2DHelper::GetPositionOnLine(const Line2D &line, const Eigen::Vector2d &p)
	{
		// project point onto the line
		return (p - line.p1).dot((line.p2 - line.p1).normalized());
	}

	double Line2DHelper::GetSqrDistance(const Line2D &line, const Eigen::Vector2d &p)
	{
		Eigen::Vector2d linePoint = line.p1;
		Eigen::Vector2d lineDirection = (line.p2 - line.p1);
		double lineLength = lineDirection.norm();
		lineDirection.normalize();

		Eigen::Vector2d currentPointVector = p - linePoint;

		// project point onto the line
		double currentPos = currentPointVector.dot(lineDirection);
		double distance = (currentPointVector - currentPos*lineDirection).norm();

		double projectedPointDistanceToLine = 0.0;
		if (currentPos < 0.0)
		{
			projectedPointDistanceToLine = currentPos;	
		}
		else if(currentPos > lineLength)
		{
			projectedPointDistanceToLine = currentPos - lineLength;
		}
		else
		{
			assert(distance >= 0.0);
			return distance * distance;
		}
		
		// the projection of the point is outside of the defined line, thus we need to calculate the distance using Phytagoras.
		return projectedPointDistanceToLine * projectedPointDistanceToLine + distance * distance;
	}

	double Line2DHelper::GetDistance(const Line2D &line, const Eigen::Vector2d &p)
	{
		return  std::sqrt(Line2DHelper::GetSqrDistance(line, p));
	}

	int Line2DHelper::ProjectVectorOnLine(const Line2D &line, const Eigen::Vector2d &p, double &return_position, double &return_distance)
	{
		Eigen::Vector2d linePoint = line.p1;
		Eigen::Vector2d lineDirection = (line.p2 - line.p1);
		double lineLength = lineDirection.norm();
		lineDirection.normalize();

		Eigen::Vector2d currentPointVector = p - linePoint;

		// project point onto the line
		return_position = currentPointVector.dot(lineDirection);
		return_distance = (p - (linePoint + return_position * lineDirection)).norm();
		return 0;
	}

	int Algorithms2D::ScaleToScreenCoordinates(const cv::Point2f &input, cv::Point2f &result, size_t image_width, size_t image_height)
	{
		float imageWidth = static_cast<float>(image_width);
		float imageHeight = static_cast<float>(image_height);
		result = cv::Point2f(input.x * imageWidth, input.y * imageHeight);
		return 0;
	}

	int Algorithms2D::ScaleToScreenCoordinates(const Eigen::Vector2d &input, cv::Point2f &result, size_t image_width, size_t image_height)
	{
		float imageWidth = static_cast<float>(image_width);
		float imageHeight = static_cast<float>(image_height);
		result = cv::Point2f(static_cast<float>(input.x()) * imageWidth,  static_cast<float>(input.y()) * imageHeight);
		return 0;
	}

	int Algorithms2D::ScaleToScreenCoordinates(const Eigen::Vector2d &input, Eigen::Vector2d &result, size_t image_width, size_t image_height)
	{
		double imageWidth = static_cast<double>(image_width);
		double imageHeight = static_cast<double>(image_height);
		result = Eigen::Vector2d(input.x() * imageWidth, input.y() * imageHeight);
		return 0;
	}


	int Algorithms2D::ScaleToScreenCoordinates(const std::vector<cv::Point2f> &input_points, std::vector<cv::Point2f> &result, size_t image_width, size_t image_height)
	{
		float imageWidth = static_cast<float>(image_width);
		float imageHeight = static_cast<float>(image_height);
		for(auto itr = input_points.begin(); itr != input_points.end(); itr++)
		{
			result.push_back(cv::Point2f(itr->x * imageWidth, itr->y * imageHeight));
		}
		return 0;
	}

	cv::Mat Algorithms2D::GetContoursUsingCanny(const cv::Mat &input, int threshold)
	{
		const int ratio = 3;
		const int kernel_size = 3;
		/// Reduce noise with a kernel 3x3
		cv::Mat detectedEdges;
		cv::blur(input, detectedEdges, cv::Size(3,3));

		/// Canny detector
		cv::Canny(detectedEdges, detectedEdges, threshold, threshold*ratio, kernel_size );

		return detectedEdges;
		}

	cv::Mat Algorithms2D::GetContoursUsingSobel(const cv::Mat &image_greyscale)
	{
		cv::Mat gradX, gradY, absGradX, absGradY, result;
		cv::Sobel(image_greyscale, gradX, CV_16S, 1, 0);
		cv::convertScaleAbs(gradX, absGradX);

		cv::Sobel(image_greyscale, gradY, CV_16S, 0, 1);
		cv::convertScaleAbs(gradY, absGradY);
	
		cv::Mat cameraImageEdgesSmall;
		cv::addWeighted(absGradX, 0.5, absGradY, 0.5, 0, result);
		return result;
	}

	cv::Mat Algorithms2D::Dilate(const cv::Mat &mask, int dilation_size)
	{
		
		cv::Mat element = getStructuringElement(cv::MORPH_RECT,
                                    cv::Size(2*dilation_size + 1, 2*dilation_size + 1),
                                    cv::Point(dilation_size, dilation_size) );
			

		cv::Mat dilatedMask;
	
		cv::dilate(mask, dilatedMask, element);
		return dilatedMask;
	}


	cv::Mat Algorithms2D::CreateMaskFromImage(const cv::Mat &gray_image)
	{
		assert(gray_image.channels() == 1);
		cv::Mat imageBinary = gray_image > 128;
		imageBinary = 255 - imageBinary;
		return imageBinary;
	}

	int Algorithms2D::GetConvexHull(cv::InputArray points, cv::OutputArray hull)
	{
		cv::Mat pointMatrix = points.getMat();
		if(pointMatrix.cols < 3)
		{
			return -1;
		}
		
		cv::convexHull(points, hull);
		return 0;
	}

	int Algorithms2D::Copy(cv::InputArray source_mat, cv::InputOutputArray dest_mat, cv::Point position, cv::InputArray mask_mat)
	{

		cv::Mat source = source_mat.getMat();
		cv::Mat dest = dest_mat.getMatRef();


		cv::Rect copyWindow(0, 0, source.cols - std::abs(position.x), source.rows - std::abs(position.y));

		if (position.x < 0)
		{
			copyWindow.x = -position.x;
			position.x = 0;
		}
		if(position.y < 0)
		{
			copyWindow.y = -position.y;
			position.y = 0;
		}

		if(mask_mat.empty())
		{
			source(copyWindow).copyTo(dest(cv::Rect(position.x, position.y, copyWindow.width, copyWindow.height)));
		}
		else
		{
			source(copyWindow).copyTo(dest(cv::Rect(position.x, position.y, copyWindow.width, copyWindow.height)), mask_mat.getMat()(copyWindow));
		}
		return 0;
	}

	
	int Algorithms2D::FindLinesSuzukiAbe(const cv::Mat &color_image, const cv::Mat &edge_image, std::vector<Line2D> &lines)
	{
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(edge_image, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		unsigned int minNumOfPoints = 20;



		if (false)
		{
			// for debug output purpose
			cv::Mat colorImageWithEdges = color_image.clone();

			cv::RNG rng(123);
			for (unsigned int i = 0; i < contours.size(); i++)
			{
				// ignore small surfaces
				if (contours[i].size() < minNumOfPoints)
				{
					continue;
				}
				
			
				drawContours(colorImageWithEdges, contours, i, cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 2, 8, hierarchy);
			}
			cv::imshow("Detected Contours", colorImageWithEdges);
		}

		// improve contours by filtering them by size. After that, approximate them by a polygon line
		std::vector<std::vector<cv::Point>> filteredContours;
		for (unsigned int i = 0; i < contours.size(); i++)
		{
			// ignore small surfaces
			if (contours[i].size() < minNumOfPoints)
			{
				continue;
			}
			std::vector<cv::Point> newContour;

			cv::approxPolyDP(contours[i], newContour, 3.0, true);
			filteredContours.push_back(newContour);
		}


		for (auto itr = filteredContours.begin(); itr != filteredContours.end(); itr++)
		{
			if (itr->size() < 2)
			{
				continue;
			}
			for(auto contourItr = itr->begin(); contourItr != itr->end() - 1; contourItr++)
			{
				lines.push_back(grMain::Line2D(*contourItr, *(contourItr + 1)));
			}
		}



		// draw the lines
		if (false)
		{
			cv::Mat lineImage = color_image.clone();
			for (auto itr = filteredContours.begin(); itr != filteredContours.end(); itr++)
			{
				if(itr->size() < 2)
				{
					continue;
				}

				cv::RNG rng(123);

				for(unsigned int i = 0; i < itr->size() - 1; i++)
				{
					cv::line(lineImage, (*itr)[i], (*itr)[i + 1], cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)));
				}
			}
			cv::imshow("Lines", lineImage);

		}
		return 0;
	}

	/*
	int Algorithms2D::MergeLines(std::vector<Line2D> &lines, double delta, double max_angle_diff_rad)
	{

		double sqrDelta = delta*delta;
		double cosMaxAngleDiff = std::cos(max_angle_diff_rad);

		/*
		This lambda function finds the best merging candidate for a line 
		*
		auto FindMergeCandidates = [cosMaxAngleDiff, sqrDelta] (Line2D line, const std::vector<Line2D>::iterator &begin, const std::vector<Line2D>::iterator &end, std::vector<Line2D>::iterator &best_p1, int &best_p1_other_index, std::vector<Line2D>::iterator &best_p2, int &best_p2_other_index) -> int
		{
			best_p1 = end;
			best_p2 = end;
			double p1BestCosDiff = 0.0;
			double p2BestCosDiff = 0.0;

			for (auto itr = begin; itr != end; itr++)
			{
				/* there are now four cases how the lines can belong to each other:
				Case 1: l1.p2 and l2.p1 match and both direction vecs are equal
				Case 2: l1.p1 and l2.p2 match and both direction vecs are equal
				Case 3: l1.p1 and l2.p1 match and the direction vectors are mirrored
				Case 4: l1.p2 and l2.p2 match and the direction vectors are mirrored
				*
				std::vector<bool> matches(4, false);
				matches[0] = ((line.p1 - itr->p1).squaredNorm() < sqrDelta); // L1P1L2P1
				matches[1] = ((line.p2 - itr->p1).squaredNorm() < sqrDelta); // L1P1L2P2
				matches[2] = ((line.p1 - itr->p2).squaredNorm() < sqrDelta); // L1P2L2P1
				matches[3] = ((line.p2 - itr->p2).squaredNorm() < sqrDelta); // L1P2L2P2

				// ensure that only one match is done
				if (std::count(matches.begin(), matches.end(), true) != 1)
				{
					continue;
				}

				double cosDiff = 0.0; 
				if (matches[1] || matches[2])
				{
					// check if cos of angle is closer to 1 than 
					cosDiff = line.direction().dot(itr->direction()) - cosMaxAngleDiff;
				}
				else if(matches[0] || matches[3])
				{
					// check if angle is 180 degree
					cosDiff = -line.direction().dot(itr->direction()) - cosMaxAngleDiff;
				}

				if(cosDiff > 0.0)
				{
					// check if it belongs to the line begin or end point
					if (matches[0] || matches[2])
					{
						// the new line segment belongs to p1 of the current line	
						if(cosDiff > p1BestCosDiff)
						{
							best_p1 = itr;
							p1BestCosDiff = cosDiff;
							if (matches[0])
							{
								best_p1_other_index = 0;
							}
							best_p1_other_index = 1;
						}
					}
					else
					{
						// belongs to p2
						if(cosDiff > p2BestCosDiff)
						{
							best_p2 = itr;
							p2BestCosDiff = cosDiff;
							if (matches[1])
							{
								best_p2_other_index = 0;
							}
							best_p2_other_index = 1;
						}
					}
				}
			}
			return 0;
		};

		for (auto itr = lines.begin(); itr < lines.end() - 1; itr++)
		{

			std::vector<Line2D>::iterator bestLineP1 = lines.end(); 
			std::vector<Line2D>::iterator bestLineP2 = lines.end();
			int bestLineP1Index = 0;
			int bestLineP2Index = 0;

			FindMergeCandidates(*itr, itr + 1, lines.end(), bestLineP1, bestLineP1Index, bestLineP2, bestLineP2Index);

			// merge the current line with the other, found matches and delete the last item of the list
			std::vector<std::vector<Line2D>::iterator> deletionIndices;

			if(bestLineP1 != lines.end())
			{
				if(0 == bestLineP1Index)
				{
					// line p1 and best line->p1 are equal
					itr->p1 = bestLineP1->p2;
				}
				else
				{	
					// line p1 and bestline p2 are equal
					itr->p1 = bestLineP1->p1;
				}

				deletionIndices.push_back(bestLineP1);
			}

			if(bestLineP2 != lines.end())
			{
				if(0 == bestLineP2Index)
				{
					itr->p2 = bestLineP2->p2;
				}
				else
				{	
					itr->p2 = bestLineP2->p1;
				}

				deletionIndices.push_back(bestLineP2);
			}

			// ... and delete
			if(deletionIndices.size() > 0)
			{
				std::sort(deletionIndices.begin(), deletionIndices.end());
				while(deletionIndices.size() > 0)
				{
					lines.erase(deletionIndices.back());
					deletionIndices.pop_back();
				}
			}

		}

		// filter out all lines with length 0 - even though this should not happen
		for(int i = lines.size(); i != 0; i--)
		{
			if (lines[i - 1].length() < 0.0001)
			{
				lines.erase(lines.begin() + i - 1);
			}
		}
		return 0;
	}*/

	int Algorithms2D::MergeLines(std::vector<Line2D> &lines, double delta, double max_angle_diff_rad)
	{

		double sqrDelta = delta*delta;
		double cosMaxAngleDiff = std::cos(max_angle_diff_rad);

		/*
		This lambda function finds the best merging candidate for a line 
		*/
		auto FindMergeCandidates = [cosMaxAngleDiff, delta, sqrDelta] (Line2D &line, const std::vector<Line2D>::iterator &begin, const std::vector<Line2D>::iterator &end, std::vector<std::vector<Line2D>::iterator> &lines_to_be_deleted) -> int
		{
			for (auto itr = begin; itr != end; itr++)
			{
				// check if p1 or p2 of the the other line are close enough to the current line
				if((Line2DHelper::GetSqrDistance(line, itr->p1) < sqrDelta) || (Line2DHelper::GetSqrDistance(line, itr->p2) < sqrDelta))
				{
					// lines are close enough together, check angle
					double cosDiff = std::abs(line.direction().dot(itr->direction())) - cosMaxAngleDiff;

					if(cosDiff > 0.0)
					{
						double lineP1 = 0.0;
						double lineP2 = line.length();
						double otherP1 = 0.0;
						double otherP2 = 0.0;
						double distanceP1 = 0.0;
						double distanceP2 = 0.0;

						// we got the best results yet, project all points onto the line
						Line2DHelper::ProjectVectorOnLine(line, itr->p1, otherP1, distanceP1);
						Line2DHelper::ProjectVectorOnLine(line, itr->p2, otherP2, distanceP2);

						if (otherP1 > otherP2)
						{
							// arrange in ascending sequence
							std::swap(otherP1, otherP2);
							std::swap(distanceP1, distanceP2);
							std::swap(itr->p1, itr->p2);
						}

						std::vector<double*> sortList;
						sortList.reserve(4);
						sortList.push_back(&lineP1);
						sortList.push_back(&lineP2);
						sortList.push_back(&otherP1);
						sortList.push_back(&otherP2);
						
						// sort by their length
						std::sort(sortList.begin(), sortList.end(), [] (double* i, double *j) -> bool
						{
							return(*i < *j);
						});

						// the other line can be deleted.
						lines_to_be_deleted.push_back(itr);

						if (sortList[0] == &lineP1 && sortList[3] == &lineP2)
						{
							// the line to compare is smaller than the current one, so we can skip the result.
						}
						else if(sortList[2] == &lineP1)
						{
							// replace p1 of the current line with p1 of the other line (current p1 and other p2 are close together)
							assert(std::abs(otherP2 - lineP1) < delta);
							line.p1 = itr->p1;
						}
						else if(sortList[1] == &lineP2)
						{
							// replace p2 of the current line with p2 of the other line (current p2 and other p1 are close together)
							assert(std::abs(otherP1 - lineP2) < delta);
							line.p2 = itr->p2;
						}
						else if(sortList[0] == &lineP1 && sortList[2] == &lineP2)
						{
							// left part of line 2 is within line 1
							line.p2 = itr->p2;
						}
						else if(sortList[0] == &otherP1 && sortList[2] == &otherP2)
						{
							line.p1 = itr->p1;
						}
						else
						{
							// last possible case - the current line is smaller than the other line. Copy over the values of the other line and delete it
							assert(sortList[0] == &otherP1);
							assert(sortList[3] == &otherP2);
							line.p1 = itr->p1;
							line.p2 = itr->p2;
						}

					}

				}
			}
			return 0;
		};

		for (auto itr = lines.begin(); itr < lines.end() - 1; itr++)
		{

			std::vector<Line2D>::iterator bestLineP1 = lines.end(); 
			std::vector<Line2D>::iterator bestLineP2 = lines.end();
			int bestLineP1Index = 0;
			int bestLineP2Index = 0;

			std::vector<std::vector<Line2D>::iterator> linesToBeDeleted;


			// merge the line with others...
			FindMergeCandidates(*itr, itr + 1, lines.end(), linesToBeDeleted);

			// ...and delete the lines which are not important anymore
			if(linesToBeDeleted.size() > 0)
			{
				std::sort(linesToBeDeleted.begin(), linesToBeDeleted.end());
				

				while(linesToBeDeleted.size() > 0)
				{
					*linesToBeDeleted.back() = lines.back();
					lines.pop_back();
					linesToBeDeleted.pop_back();
				}
			}

		}

		return 0;
	}
	
	int Algorithms2D::FindConnectingLines(std::vector<Line2D> &lines, std::vector<Eigen::Vector2d> image_points, std::vector<std::pair<size_t, size_t>> &matches, double max_distance_image_point_line, double max_distance_line_line, double line_acceptance_angle_rad, size_t num_of_iterations)
	{
		double sqrImagePointDistance = max_distance_image_point_line*max_distance_image_point_line;
		double sqrLineLineDistance = max_distance_line_line*max_distance_line_line;

		auto AddMatchToList = [&matches](std::pair<size_t, size_t> new_match) -> int
		{
			for (auto itr = matches.begin(); itr != matches.end(); itr++)
			{
				if ((itr->first == new_match.first && itr->second == new_match.second) || 
				    (itr->second == new_match.first && itr->first == new_match.second))
				{
					// match does already exist
					return -1;
				}
			}

			if(new_match.first > new_match.second)
			{
				std::swap(new_match.first, new_match.second);
			}
			matches.push_back(new_match);
			return 0;
		};

		/*
		if(true)
		{
			// debug output
			std::ostringstream windowNameStream;
			windowNameStream << "Initial Stage";
			cv::Mat currentImage = editor->camLocalImageUndistorted.clone();
			cv::RNG rng(123);
			for (auto itr = lines.begin(); itr != lines.end(); itr++)
			{
				cv::line(currentImage, ToCVPoint(itr->p1), ToCVPoint(itr->p2), cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 2);
			}

			cv::imshow(windowNameStream.str(), currentImage);
		}
		*/
		for (size_t i = 0; i != num_of_iterations; i++)
		{
			if (lines.size() == 0)
			{
				return 0;
			}

			// first of all, simplify the line set
			Algorithms2D::MergeLines(lines, max_distance_line_line, line_acceptance_angle_rad);
			
			std::vector<std::vector<Line2D>::iterator> linesToRemove;

			// find matching points
			for (auto itr = lines.begin(); itr != lines.end(); itr++)
			{
				std::vector<std::pair<std::vector<Eigen::Vector2d>::const_iterator, double>> currentMatches;
				for (auto pItr = image_points.begin(); pItr != image_points.end(); pItr++)
				{
					// search if the point is close enough to the line
					if (Line2DHelper::GetSqrDistance(*itr, *pItr) < sqrImagePointDistance)
					{
						currentMatches.push_back(std::pair<std::vector<Eigen::Vector2d>::const_iterator, double>(pItr, Line2DHelper::GetPositionOnLine(*itr, *pItr)));
					}
				}

				// if more than one point is close enought to the line, we have at least one pair
				if (currentMatches.size() > 1)
				{
					// sort the matches depending on their position on the line.
					std::sort(currentMatches.begin(), currentMatches.end(), [](const std::pair<std::vector<Eigen::Vector2d>::const_iterator, double> &point1, const std::pair<std::vector<Eigen::Vector2d>::const_iterator, double> &point2)
					{
						return point1.second < point2.second;
					});
					
					// store the matches
					for (auto matchItr = currentMatches.begin(); matchItr != currentMatches.end() - 1; matchItr++)
					{
						size_t firstItr = (*matchItr).first - image_points.begin();
						size_t secondItr = (*(matchItr + 1)).first - image_points.begin();
						AddMatchToList(std::pair<size_t, size_t>(firstItr, secondItr));
					}
					//...and mark the line for deletion
					linesToRemove.push_back(itr);
					continue;
				}

			}

			// delete all found lines. Can probably solved better but internet is currently not working. Check the erase remove idiom	
			std::sort(linesToRemove.begin(), linesToRemove.end());
			while(linesToRemove.size() > 0)
			{
				//overwrite item with last of list
				*linesToRemove.back() = lines.back();
				linesToRemove.pop_back();
				lines.pop_back();
			}

			// expand all other lines
			for (auto itr = lines.begin(); itr != lines.end(); itr++)
			{
				// change the line length
				itr->p2 += itr->direction() * 0.5 * max_distance_line_line;
				itr->p1 -= itr->direction() * 0.5 * max_distance_line_line;
			}

			
			/*
			if(true)
			{
				// debug output
				std::ostringstream windowNameStream;
				windowNameStream << "Iteration Step " << (i + 1);
				cv::Mat currentImage = editor->camLocalImageUndistorted.clone();
				cv::RNG rng(123);
				for (auto itr = lines.begin(); itr != lines.end(); itr++)
				{
					cv::line(currentImage, ToCVPoint(itr->p1), ToCVPoint(itr->p2), cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 2);
				}

				cv::imshow(windowNameStream.str(), currentImage);
			}
			*/
		}
		return 0;
	}

	int Algorithms2D::FindCircles(const cv::Mat &input_image)
	{
		// Convert it to gray
		cv::Mat imageGray;

		cv::cvtColor(input_image, imageGray, cv::COLOR_BGR2GRAY);

		/// Reduce the noise so we avoid false circle detection
		GaussianBlur(imageGray, imageGray, cv::Size(9, 9), 2, 2);

		std::vector<cv::Vec3f> circles;

		cv::imshow("gray", imageGray);
		/// Apply the Hough Transform to find the circles
		cv::HoughCircles(imageGray, circles, cv::HOUGH_GRADIENT, 1, imageGray.rows / 15, 100, 70, 0, 0 );
		
		cv::Mat resultImage = input_image.clone();

		/// Draw the circles detected
		for (size_t i = 0; i < 2; i++) // circles.size(); i++)
		{
			cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);
			// circle center
			cv::circle(resultImage, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
			// circle outline
			circle(resultImage, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
		}

		/// Show your results
		cv::imshow( "Hough Circle Transform Demo", resultImage);
		return 0;
	}

	int Algorithms2D::ExtractEllipseToImage(const arMath::Ellipse &ellipse, const cv::Mat &source_image, cv::Mat &output_buffer, double RotationAngle)
	{
		// transform from a source rectangle to a destination rectangle.
		/*
		Start by calculating the unit vectors of the destination image in the source image (dest x and y vectors
		in the source image, where they might be scaled and rotated. For the first step, we only take the
		rotation into account.
		*/
		double angle;
		if (0 != ellipse.Angle(angle))
		{
			return -1;
		}

		Eigen::Vector2d vectorX, vectorY;
		vectorX << cos(angle), sin(angle);
		vectorY << -1.0 * sin(angle), cos(angle);


		/*
		In the second step, we scale the vectors approprately.
		*/
		double majorAxis = 0.0;
		double minorAxis = 0.0;

		ellipse.MajorAxis(majorAxis);
		ellipse.MinorAxis(minorAxis);

		Eigen::Vector2d center;
		ellipse.Center(center);

		double coldx = center.x();
		double coldy = center.y();
		// new cx

		double cx = center.x();
		double cy = center.y();


		double imageWidth = static_cast<double>(output_buffer.cols);
		double imageHeight = static_cast<double>(output_buffer.rows);

		vectorX *= 2.0 * majorAxis / (static_cast<double>(output_buffer.cols)); // / static_cast<double>(source_image.rows);
		vectorY *= 2.0 * minorAxis / imageHeight;

		const double cosRotAngle = cos(RotationAngle);
		const double sinRotAngle = sin(RotationAngle);

		auto GetSourceCoordinates = [&](int x, int y, cv::Point &result) -> int
		{
			// transform and rotate coordinates
			const double srcX = static_cast<double>(x) - 0.5 * imageWidth;
			const double srcY = static_cast<double>(y)-0.5 * imageWidth;

			const double rotatedSrcX = cosRotAngle * srcX - sinRotAngle * srcY;
			const double rotatedSrcY = cosRotAngle * srcY + sinRotAngle * srcX;

			auto resultCoords = center 
				+ (rotatedSrcX) * vectorX
				+ (rotatedSrcY) * vectorY;


			result.x = static_cast<int>(resultCoords.x());
			result.y = static_cast<int>(resultCoords.y());
			if (result.x < 0 || result.x >= source_image.cols
				|| result.y < 0 || result.y >= source_image.rows)
			{
				return -1;
			}
			return 0;
		};


		// now, transform all points
		for (int x = 0; x != output_buffer.cols; x++)
		{
			for (int y = 0; y != output_buffer.rows; y++)
			{
				cv::Point coordinates;
				if (0 == GetSourceCoordinates(x, y, coordinates))
				{
					if (3 == source_image.channels())
					{
						output_buffer.at<cv::Vec3b>(y, x) = source_image.at<cv::Vec3b>(coordinates.y, coordinates.x);
					}
					else
					{
						output_buffer.at<uchar>(y, x) = source_image.at<uchar>(coordinates.y, coordinates.x);
					}


				}
			}
		}
		return 0;
	}

	cv::Mat Algorithms2D::ImageRotate(const cv::Mat &src, const cv::Point &center, double rotation_angle)
	{
		auto  rotationMatrix = cv::getRotationMatrix2D(center, rotation_angle * 180.0 / M_PI, 1.0);

		cv::Mat dst;
		cv::warpAffine(src, dst, rotationMatrix, src.size());
		return dst;
	}

	cv::Mat Algorithms2D::CircularToLinear(const cv::Mat &src, cv::Point const &center, double radius_min, double radius_max, size_t angle_intervals)
	{
		size_t width = static_cast<size_t>(radius_max - radius_min);

		// create an rectangular output image.
		cv::Mat outputArray = cv::Mat(cv::Size(width, angle_intervals), src.type());

		const double angle_step = 2.0 * M_PI / static_cast<double>(angle_intervals);

		// for each pixel, search the correspondence on the source image.
		for (size_t x = 0; x != outputArray.cols; ++x)
		{
			for (size_t y = 0; y != outputArray.rows; ++y)
			{
				const double radius = radius_min + x;
				const double angle = static_cast<double>(y)* angle_step;
				size_t src_x = static_cast<int>(center.x + cos(angle) * radius);
				size_t src_y = static_cast<int>(center.y + sin(angle) * radius);
				
				outputArray.at<uchar>(y, x) = src.at<uchar>(src_y, src_x);
			}
		}

		return outputArray;
	}

	cv::Mat Algorithms2D::CircularToLinearAverage(const cv::Mat &src, cv::Point const &center, double radius_min, double radius_max)
	{
		size_t width = static_cast<size_t>(radius_max - radius_min) + 1;
		
		// transform all pixels from the color image into a linear space
		std::vector<std::vector<uchar>> lin_space_bins;
		lin_space_bins.resize(width);

		for (size_t x = 0; x != src.cols; ++x)
		{
			for (size_t y = 0; y != src.rows; ++y)
			{
				double distance = cv::norm(cv::Point(x, y) - center);
				if (distance > radius_max || distance < radius_min)
				{
					continue;
				}

				// add the current pixel to the bin for the current image
				size_t lin_space_element = static_cast<size_t>(distance - radius_min);
				lin_space_bins[lin_space_element].push_back(src.at<uchar>(y, x));
			}
		}

		// create an rectangular output image.
		cv::Mat outputArray = cv::Mat(cv::Size(width, 10), src.type());

		// for each bin, calculate an average color value
		for (size_t i = 0; i != lin_space_bins.size(); ++i)
		{
			uchar currentSum = static_cast<uchar>(std::accumulate(lin_space_bins[i].begin(), lin_space_bins[i].end(), 0) / lin_space_bins[i].size());
			for (size_t y = 0; y != outputArray.rows; ++y)
			{
				outputArray.col(i).row(y) = currentSum;
			}
		}

		return outputArray;
	}

	void Algorithms2D::match(cv::Mat const &image, cv::Mat const &templ, cv::Mat1b const &templ_mask, cv::Point const &position, int method, double &corr)
	{
		bool use_mask = false;
		if (templ_mask.size() == templ.size())
		{
			use_mask = true;
		}

		corr = 0.0;
		size_t num_of_pixels = 1;

		for (size_t x = 0; x != templ.cols; ++x)
		{
			for (size_t y = 0; y != templ.rows; ++y)
			{
				if (templ_mask.at<uchar>(x, y) == 0)
				{
					continue;
				}

				double templValue = static_cast<double>(templ.at<uchar>(y, x));
				double imgValue = static_cast<double>(image.at<uchar>(position.y + y, position.x + x));
				++num_of_pixels;

				// calculate correlation
				switch (method)
				{
				case cv::TM_SQDIFF:
					corr += std::pow(templValue - imgValue, 2.0);
					break;
				case cv::TM_CCORR:
					corr += templValue * imgValue;
					break;
				default:
					assert(false);
					return;
				}
			}
		}
		corr /= static_cast<double>(num_of_pixels);
	}
	
	double Algorithms2D::ImageCorrelation(const cv::Mat &img1, const cv::Mat &img2, cv::Mat1b mask)
	{
		size_t sizeX = img1.cols;
		size_t sizeY = img1.rows;
		double corr = 0.0;
		size_t numOfPixels = 0;
		bool useMask = false;
		if (mask.size() == img1.size())
		{
			useMask = true;
		}

		for (size_t x = 0; x != sizeX; x++)
		{
			for (size_t y = 0; y != sizeY; y++)
			{
				if (useMask && mask.at<uchar>(y, x) == 0)
				{
					continue;
				}

				numOfPixels++;
				double offset = static_cast<double>(img1.at<uchar>(y, x)) - static_cast<double>(img2.at<uchar>(y, x));
				corr += offset * offset;
			}
		}
		corr /= static_cast<double>(numOfPixels);
		return corr;
	}



	cv::Scalar Algorithms2D::GetAverageColorValueAtPosition(const cv::Mat &sourceImage, const Eigen::Vector2i &position, int radius)
	{
		cv::Rect copyRect(position.x() - radius, position.y() - radius, 2 * radius + 1, 2 * radius + 1);

		int centerX = copyRect.width / 2;
		int centerY = copyRect.height / 2;

		// check boundaries of image 
		if (0 > copyRect.x)
		{
			centerX += copyRect.x;
			copyRect.width += copyRect.x;
			copyRect.x = 0;
		}
		if (0 > copyRect.y)
		{
			centerY += copyRect.y;
			copyRect.height += copyRect.y;
			copyRect.y = 0;
		}
		if (copyRect.x + copyRect.width > sourceImage.size().width)
		{
			copyRect.width = sourceImage.size().width - copyRect.x;
		}
		if (copyRect.y + copyRect.height > sourceImage.size().height)
		{
			copyRect.height = sourceImage.size().height - copyRect.y;
		}
		assert(0 < copyRect.width && 0 < copyRect.height);

		// the average color value is calculated by multiplying the extracted part of the image with a bit mask
		cv::Mat imageExtract = sourceImage(copyRect);
		cv::Mat1b mask(imageExtract.rows, imageExtract.cols);

		// create circle mask
		for (int i = 0; i != mask.rows; i++)
		{
			for (int k = 0; k != mask.cols; k++)
			{
				int diffX = centerX - k;
				int diffY = centerY - i;

				if (diffX * diffX + diffY * diffY > radius * radius)
				{
					mask.at<uchar>(i, k) = 0;
				}
				else
				{
					mask.at<uchar>(i, k) = 255;
				}
			}
		}

		return  cv::mean(imageExtract, mask);
	}

	int Algorithms2D::FindEllipses(const cv::Mat &edge_image, const EllipseDetectionAttributes& attributes, std::vector<arMath::Ellipse> &found_ellipses, std::vector<std::vector<Eigen::Vector2d>> &found_contours)
	{
		size_t minNumOfPoints = attributes.minNumOfPoints;

		double minAxisLength = attributes.minAxisLength;
		double maxAxisLength = attributes.maxAxisLength;
		double maxFittingError = attributes.maxFittingError;

		int max_thresh = 255;
		cv::RNG rng(12345);

		
		
		std::vector<std::vector<cv::Point>> contoursCV;
		std::vector<cv::Vec4i> hierarchy;
 
		/// Find contours
		cv::findContours(edge_image, contoursCV, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
		std::vector<std::vector<Eigen::Vector2d>> contours;
		
		for(size_t i = 0; i != contoursCV.size(); i++)
		{
			if( contoursCV[i].size() > static_cast<unsigned int>(minNumOfPoints))
			{ 
				contours.push_back(ToVectorEVector2d(contoursCV[i]));
			}
		}

		std::vector<arMath::Ellipse> foundEllipses;
		std::vector<std::vector<Eigen::Vector2d>> currentContours;
		
		double maxPrefitError = maxFittingError;

		for(unsigned int i = 0; i < contours.size(); i++ )
		{ 

			arMath::Ellipse result;
			double fittingError = 0.0;
			auto currentPointSet = contours[i];
			if (0 != Math::FitEllipseLeastSquares(currentPointSet, result, fittingError))
			{
				continue;
			}

			if(false == arMath::EllipseHelper::IsGoodEllipseFit(result, fittingError, currentPointSet, minAxisLength, maxAxisLength, maxPrefitError))
			{
				continue;
			}
				
			foundEllipses.push_back(result);
			currentContours.push_back(contours[i]);
		}



		// just copy the data over
		for (size_t i = 0; i != foundEllipses.size(); i++)
		{
			found_ellipses.push_back(foundEllipses[i]);
			found_contours.push_back(currentContours[i]);
		}
		return 0;
	}

	bool Algorithms2D::ArePolygonsEqual(const std::vector<unsigned int> &polygon1, const std::vector<unsigned int> &polygon2)
	{
		if(polygon1.size() != polygon2.size())
		{
			return false;
		}

		for(auto pItr = polygon1.begin(); pItr != polygon1.end(); pItr++)
		{
			if(std::find(polygon2.begin(), polygon2.end(), *pItr) == polygon2.end())
			{
				return false;
			}
		}
		return true;
	}
	CircularMotion Algorithms2D::PolygonOrientation(const std::vector<Eigen::Vector2d> &input_points, const grMain::IndicesPolygon &input_polygon)
	{
		double sum = 0.0;
		// find the polygon orientation.
		for(unsigned int i = 0; i != input_polygon.pointIndices.size(); i++)
		{
			unsigned int k = (i + 1) % input_polygon.pointIndices.size();
			{
				sum += (input_points[input_polygon.pointIndices[k]].x() - input_points[input_polygon.pointIndices[i]].x()) * 
					(input_points[input_polygon.pointIndices[k]].y() + input_points[input_polygon.pointIndices[i]].y());
			}
		}

		if(sum >= 0.0)
		{
			return CIRCULAR_MOTION_CW;
		}
		return CIRCULAR_MOTION_CCW;
	}

	std::vector<Eigen::Vector2d> Algorithms2D::ShrinkPolygon(const std::vector<Eigen::Vector2d> &polygon, double distance)
	{
		std::vector<Eigen::Vector2d> shrinkedPolygon;

		for(unsigned int currentIndex = 0; currentIndex != polygon.size(); currentIndex++)
		{
			unsigned int lastIndex = (currentIndex + polygon.size() - 1) % polygon.size();
			unsigned int nextIndex = (currentIndex + 1) % polygon.size();

			// calculate the angle
			Eigen::Vector2d v1 = polygon[nextIndex] - polygon[currentIndex];
			Eigen::Vector2d v2 = polygon[lastIndex] - polygon[currentIndex];


			double angle = arMath::CompleteAngle2D(v1, v2) / 2.0;
			assert(angle < 3.2);
			assert(angle > -0.01);
			// take half the angle and rotate the first vector by it. This rotated vector then points inwards of the polygon at exact the middle between the two edges.
			Eigen::Vector2d pointShiftVector = Eigen::Vector2d( std::cos(angle) * v2.x() + std::sin(angle) * v2.y(),
															    -std::sin(angle) * v2.x() + std::cos(angle) * v2.y());

			// and shift it by the given offset.
			pointShiftVector.normalize();
			shrinkedPolygon.push_back(polygon[currentIndex] + distance * pointShiftVector);
		}
		return shrinkedPolygon;
	}



	std::vector<Eigen::Vector2d> Algorithms2D::ScalePolygon(const std::vector<Eigen::Vector2d> &polygon, double scale)
	{
		Eigen::Vector2d centroid = Math::FindCentroid(polygon);
		std::vector<Eigen::Vector2d> scaledPolygon;

		for(unsigned int currentIndex = 0; currentIndex != polygon.size(); currentIndex++)
		{
			Eigen::Vector2d currentVector = polygon[currentIndex] - centroid;
			scaledPolygon.push_back(centroid + scale * currentVector);
		}
		return scaledPolygon;
	}

	bool Algorithms2D::IsPointInsidePolygon(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector2d &point)
	{
		bool isInside = false;

		size_t i, j;
		for (i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) 
		{
			if ( ((polygon[i].y() > point.y()) != (polygon[j].y() > point.y())) && (point.x() < (polygon[j].x() - polygon[i].x()) * (point.y() - polygon[i].y()) / (polygon[j].y() - polygon[i].y()) + polygon[i].x()))
			{
				isInside = !isInside;
			}
		}
		return isInside;
	}
}
