#ifndef I_MIXED_CAMERA_MANAGER_H
#define I_MIXED_CAMERA_MANAGER_H

#include <opencv2/core.hpp>

#ifdef APPLE
#define SCANNERLIB_API
#else
#define SCANNERLIB_API _declspec(dllexport)
#endif

#ifndef _ColorSpacePoint_
#define _ColorSpacePoint_
typedef struct _ColorSpacePoint
{
	float X;
	float Y;
} 	ColorSpacePoint;
#endif // _ColorSpacePoint_

class IMixedCameraManager
{
public:
	SCANNERLIB_API IMixedCameraManager() = default;
	virtual SCANNERLIB_API ~IMixedCameraManager() = default;
	virtual SCANNERLIB_API bool init() = 0;
	virtual SCANNERLIB_API bool init(std::string filePath = "") = 0;
	virtual SCANNERLIB_API void stop() = 0;
	virtual SCANNERLIB_API cv::Mat getColorFrame(int delayMS) = 0;
	virtual SCANNERLIB_API cv::Mat getDepthFrame(int delayMS) = 0;
	virtual SCANNERLIB_API cv::Mat getAveragedDepthFrame(int numFramesAveraged = -1, std::vector<ColorSpacePoint>* colorPoints = nullptr) = 0;
	virtual SCANNERLIB_API double getTableAt(double x, double y) = 0;
	virtual SCANNERLIB_API cv::Point3d get3DFromDepthAt(double x, double y, double depth) = 0;
};

#endif // I_MIXED_CAMERA_MANAGER_H