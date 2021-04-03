#ifndef CANNY_FILTER_H
#define CANNY_FILTER_H

#include <opencv2/imgproc.hpp>
#include "CVTask.h"

class ContourDetector :
	public CVTask
{
public:
	float threshold1;
	float threshold2;

	std::vector<std::vector<cv::Point>> contours;
	std::vector<std::vector<cv::Point>> hull;
	std::vector<double> areas;

	ContourDetector(float threshold1, float threshold2);

    void process(cv::InputOutputArray depthMat, cv::InputOutputArray colorMat) override;
};

#endif // CANNY_FILTER_H
