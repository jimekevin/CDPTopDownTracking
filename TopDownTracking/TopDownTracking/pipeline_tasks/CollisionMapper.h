#ifndef COLLISION_MAPPER_H
#define COLLISION_MAPPER_H

#include "Pipeline.h"
#include "pipeline_tasks/ContourDetector.h"
#include "Map.h"
#include "KinectManager_Windows.h"

class CollisionMapper :
	public Pipeline::Task
{
private:
	ContourDetector *cd;
	Map *map;
	CameraSpacePoint *color2camera;

public:
	CollisionMapper(ContourDetector *cd, Map *map, CameraSpacePoint *color2camera);

	void apply(cv::Mat& depthMat, cv::Mat& rgbMat);
};

#endif // COLLISION_MAPPER_H