#include "Map.h"

#include <cassert>

#define MAKE_RECT(x, y, z, w, h, r, g, b) { \
	{ -(x) - w/2, y - h/2, z, r, g, b }, \
	{ -(x) + w/2, y - h/2, z, r, g, b }, \
	{ -(x) + w/2, y + h/2, z, r, g, b }, \
	{ -(x) - w/2, y - h/2, z, r, g, b }, \
	{ -(x) - w/2, y + h/2, z, r, g, b }, \
	{ -(x) + w/2, y + h/2, z, r, g, b }, \
}

//const float vertices[][6][6] = {
//	MAKE_RECT(0.2f, 0.25f, 0.0f, 1.2f, 0.65f, 0.5f, 0.5f, 0.5f), // Plane
//	MAKE_RECT(-0.0f, 0.2f, -0.0001f, 0.2f, 0.1f, 0.0f, 0.7f, 0.0f), // Building 1
//	MAKE_RECT(-0.3f, 0.3f, -0.0001f, 0.1f, 0.2f, 0.0f, 0.7f, 0.0f), // Building 2
//	MAKE_RECT(0.5f, 0.4f, -0.0001f, 0.1f, 0.15f, 0.0f, 0.7f, 0.0f), // Building 3
//	MAKE_RECT(0.6f, 0.15f, -0.0001f, 0.15f, 0.15f, 0.0f, 0.7f, 0.0f), // Building 4
//};

Map::Map() {
	buildings[0] = Building{ MAKE_RECT(0.2f, 0.25f, 0.0f, 1.2f, 0.65f, 0.5f, 0.5f, 0.5f) }; // Plane

	//buildings[1] = Building{ MAKE_RECT(-0.0f, 0.2f, -0.0001f, 0.2f, 0.1f, 0.0f, 0.7f, 0.0f) }; // Building 1
	//buildings[2] = Building{ MAKE_RECT(-0.3f, 0.3f, -0.0001f, 0.1f, 0.2f, 0.0f, 0.7f, 0.0f) }; // Building 2
	//buildings[3] = Building{ MAKE_RECT(0.5f, 0.4f, -0.0001f, 0.1f, 0.15f, 0.0f, 0.7f, 0.0f) }; // Building 3
	//buildings[4] = Building{ MAKE_RECT(0.6f, 0.15f, -0.0001f, 0.15f, 0.15f, 0.0f, 0.7f, 0.0f) }; // Building 4

	buildings[1] = Building{ MAKE_RECT(0.6f, 0.15f, -0.0001f, 0.15f, 0.15f, 0.0f, 0.7f, 0.0f) }; // Building 4
	//buildings[1] = Building{ MAKE_RECT(0.1f, 0.15f, -0.0001f, 0.15f, 0.15f, 0.0f, 0.7f, 0.0f) }; // Building 4
	buildings[2] = Building{ MAKE_RECT(0.3f, 0.05f, -0.0001f, 0.15f, 0.15f, 0.0f, 0.7f, 0.0f) }; // Building 4

	countBuildings = 2;
}

Map::~Map() {
}

void Map::getXYWH(int id, Map::XYWH *xywh) {
	xywh->w = buildings[id].tile[0][Building::Index::X] -
		buildings[id].tile[5][Building::Index::X];
	xywh->w = xywh->w < 0 ? xywh->w * (-1) : xywh->w;
	xywh->h = buildings[id].tile[0][Building::Index::Y] -
		buildings[id].tile[5][Building::Index::Y];
	xywh->h = xywh->h < 0 ? xywh->h * (-1) : xywh->h;
	// Irgendwie so, mal weiter schauen
	xywh->x = buildings[id].tile[0][Building::Index::X];
	xywh->y = buildings[id].tile[0][Building::Index::Y];
}

int Map::buildingsCollideWithPoint(float x, float y, std::vector<int> &ids/*, CameraSpacePoint *color2camera*/) {
	for (int id = 1; id < countBuildings; id++) {
		XYWH xywh;
		getXYWH(id, &xywh);

		int index = y * COLORWIDTH + x;
		//auto p = color2camera[index];

		if (x >= xywh.x && x <= xywh.x + xywh.w && y >= xywh.y && y <= xywh.y + xywh.h) {
			ids.push_back(id);
		}
	}
	return ids.size();
}

void Map::markBuilding(unsigned int id) {
	assert(id > 0 && id < countBuildings);

	for (int i = 0; i < 6; i++) {
		buildings[id].tile[i][Building::Index::R] = 0.0f;
		buildings[id].tile[i][Building::Index::G] = 0.0f;
		buildings[id].tile[i][Building::Index::B] = 0.7f;
	}
}

void Map::unmarkBuilding(unsigned int id) {
	assert(id < countBuildings);

	for (int i = 0; i < 6; i++) {
		buildings[id].tile[i][Building::Index::R] = 0.0f;
		buildings[id].tile[i][Building::Index::G] = 0.7f;
		buildings[id].tile[i][Building::Index::B] = 0.0f;
	}
}

void Map::unmarkAllBuildings() {
	for (int id = 1; id < countBuildings; id++) {
		unmarkBuilding(id);
	}
}

const void* Map::data() {
	return &buildings;
}

const int Map::size() {
	return countBuildings * sizeof(Map::Building::Tile);
}