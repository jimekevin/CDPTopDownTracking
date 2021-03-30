#ifndef THRESHOLD_FILTER_H
#define THRESHOLD_FILTER_H

#include "Pipeline.h"

#include <cmath>

class ThresholdFilter :
	public Pipeline::Task
{
public:
	float minX = std::numeric_limits<double>::infinity();
	float maxX = std::numeric_limits<double>::infinity();
	float minY = std::numeric_limits<double>::infinity();
	float maxY = std::numeric_limits<double>::infinity();
	float minZ = std::numeric_limits<double>::infinity();
	float maxZ = std::numeric_limits<double>::infinity();

	ThresholdFilter(float minX, float maxX, float minY, float maxY, float minZ, float maxZ);

	void updateMinMax(float minX, float maxX, float minY, float maxY, float minZ, float maxZ);
	void apply(cv::Mat& depthMat, cv::Mat& rgbMat);
};

#endif // THRESHOLD_FILTER_H
