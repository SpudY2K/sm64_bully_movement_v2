#pragma once

#include "vmath.hpp"
#include <vector>

#ifndef SURFACE_H
#define SURFACE_H

class Surface {
public:
	VecVec3f vertices;
	Vec3f normal;
	float origin_offset;
	float lower_y;
	float upper_y;

	float min_x;
	float max_x;
	float min_z;
	float max_z;

	Surface(std::vector<std::vector<float>> verts) {
		set_vertices(verts);
	}

	Surface(VecVec3f verts) {
		set_vertices(verts);
	}

	Surface() {}

	void calculate_normal();
	void set_vertices(std::vector<std::vector<float>> &verts);
	void set_vertices(VecVec3f &verts);
};
#endif