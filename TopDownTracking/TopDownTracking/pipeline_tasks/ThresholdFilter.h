#ifndef TOPDOWNTRACKING_THRESHOLD_FILTER_H
#define TOPDOWNTRACKING_THRESHOLD_FILTER_H

#include "CVTask.h"

#include <cmath>

class ThresholdFilter :
	public CVTask
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
    void process(cv::InputOutputArray depthMat, cv::InputOutputArray colorMat) override;
};

#endif // TOPDOWNTRACKING_THRESHOLD_FILTER_H
