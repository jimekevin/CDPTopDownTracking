#ifndef I_MIXED_CAMERA_MANAGER_H
#define I_MIXED_CAMERA_MANAGER_H

#include <opencv2/core.hpp>

#ifdef APPLE
#define SCANNERLIB_API
#else
#define SCANNERLIB_API _declspec(dllexport)
#endif

class IMixedCameraManager
{
public:
	SCANNERLIB_API IMixedCameraManager() = default;
	virtual SCANNERLIB_API ~IMixedCameraManager() = default;

	virtual SCANNERLIB_API bool Init(const std::string& filePath = "") = 0;
	virtual SCANNERLIB_API cv::Mat GetColorFrame(int delayMS) = 0;
	virtual SCANNERLIB_API cv::Mat GetDepthFrame(int delayMS) = 0;
};

#endif // I_MIXED_CAMERA_MANAGER_H