#include "ThresholdFilter.h"

#include <cmath>

ThresholdFilter::ThresholdFilter(float minX, float maxX, float minY, float maxY, float minZ, float maxZ)
: minX(minX), maxX(maxX), minY(minY), maxY(maxY), minZ(minZ), maxZ(maxZ)
{
}

void ThresholdFilter::updateMinMax(float minX, float maxX, float minY, float maxY, float minZ, float maxZ) {
	this->minX = minX != std::numeric_limits<double>::infinity() ? minX : this->minX;
	this->maxX = maxX != std::numeric_limits<double>::infinity() ? maxX : this->maxX;
	this->minY = minY != std::numeric_limits<double>::infinity() ? minY : this->minY;
	this->maxY = maxY != std::numeric_limits<double>::infinity() ? maxY : this->maxY;
	this->minZ = minZ != std::numeric_limits<double>::infinity() ? minZ : this->minZ;
	this->maxZ = maxZ != std::numeric_limits<double>::infinity() ? maxZ : this->maxZ;
}

void ThresholdFilter::apply(cv::Mat& depthMat, cv::Mat& rgbMat) {
	for (int y = 0; y < depthMat.rows; y++) {
		auto depthRow = depthMat.ptr<cv::Vec3f>(y);
		auto rgbRow = rgbMat.ptr<cv::Vec3f>(y);
		for (int x = 0; x < depthMat.cols; x++) {
			auto &depth = depthRow[x];
			auto &rgb = rgbRow[x];

			if (depth[0] < minX || depth[0] > maxX || 
				depth[1] < minY || depth[1] > maxY || 
				depth[2] < minZ || depth[2] > maxZ
			) {
				depthRow[x][0] = 0.0f;
				depthRow[x][1] = 0.0f;
				depthRow[x][2] = 0.0f;
				
				rgbRow[x][0] = 1.0f;
				rgbRow[x][1] = 1.0f;
				rgbRow[x][2] = 1.0f;

			}
		}
	}
	/*for (int y = 0; y < rgbMat->rows; y++) {
		for (int x = 0; x < rgbMat->cols; x++) {
			auto &color = rgbMat->at<cv::Vec3f>(y, x);
			color[0] = 1.0f;
			color[1] = 0.5f;
			color[2] = 0.1f;
			rgbMat->at<cv::Vec3f>(cv::Point(x,y)) = color;
		}
	}*/
}
