#include "CollisionMapper.h"

CollisionMapper::CollisionMapper(ContourDetector *cd, Map *map/*, CameraSpacePoint *color2camera*/)
	: cd(cd), map(map), color2camera(color2camera)
{
}

void CollisionMapper::apply(cv::Mat& depthMat, cv::Mat& rgbMat) {
	if (cd->contours.size() <= 0) {
		return;
	}

	map->unmarkAllBuildings();

	// 1. Collision detection
	int i = -1;
	for (const auto &c : cd->contours) {
		i++;
		for (const auto &p : c) {
			if (cd->areas[i] > 5000 || cd->areas[i] < 10) {
				continue;
			}

			std::vector<int> ids;
			//map->buildingsCollideWithPoint((float)p.x / 512 - 0.5f, (float)p.y / 424 - 0.5f, ids);
			//map->buildingsCollideWithPoint((((float)p.x) / 512 - 0.5f) * 3.0f, (((float)p.y) / 424 - 0.5f) * 0.3f, ids);
			map->buildingsCollideWithPoint(p.x, p.y, ids, color2camera);
			for (auto const &id : ids) {
				map->markBuilding(id);
			}
		}
	}

	// 2. Mark detected object
}
