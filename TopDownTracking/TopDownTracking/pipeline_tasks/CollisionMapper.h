#ifndef COLLISION_MAPPER_H
#define COLLISION_MAPPER_H

#include "CVTask.h"
#include "ContourDetector.h"
#include "Map.h"

class CollisionMapper :
	public CVTask
{
private:
	ContourDetector *cd;
	Map *map;
	//CameraSpacePoint *color2camera;

public:
	CollisionMapper(ContourDetector *cd, Map *map/*, CameraSpacePoint *color2camera*/);

	void apply(cv::Mat& depthMat, cv::Mat& rgbMat);
};

#endif // COLLISION_MAPPER_H