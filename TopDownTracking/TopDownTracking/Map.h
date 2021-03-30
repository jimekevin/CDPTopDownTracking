#ifndef MAP_H
#define MAP_H

#include <vector>
#include "KinectManager_Windows.h"

#define MAX_BUILDINGS 50

class Map
{
private:
	unsigned int countBuildings = 0;

	struct Building {
		enum Index {
			X, Y, Z, R, G, B
		};
		//typedef typename float Tile[6][6];
        typedef float Tile[6][6];
		Tile tile;
	} buildings[MAX_BUILDINGS];

	struct XYWH {
		float x, y, w, h;
	};

	const static int COLORWIDTH = 1920;
	const static int COLORHEIGHT = 1080;

public:
	Map();
	~Map();

	void getXYWH(int id, Map::XYWH *xywh);
	int buildingsCollideWithPoint(float x, float y, std::vector<int> &ids/*, CameraSpacePoint *color2camera*/);

	void markBuilding(unsigned int id);
	void unmarkBuilding(unsigned int id);
	void unmarkAllBuildings();

	const void* data();
	const int size();
};

#endif // MAP_H
