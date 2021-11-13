#pragma once

#include "Surface.hpp"
#include "vmath.hpp"
#include <vector>

#ifndef WALLS_FLOORS_H
#define WALLS_FLOORS_H

extern std::vector<Surface> walls;
extern std::vector<Surface> floors;
extern std::vector<Surface> object_walls;
extern std::vector<Surface> object_floors;
extern std::vector<Surface> pyramid_platform_floors;
extern std::vector<Surface> track_platform_floors;
extern std::vector<Surface> track_platform_walls;

void filter_walls_and_floors(float y_pos);
void add_track_platform(Vec3f &track_platform_position);
void add_pyramid_platform(Vec3f &pyramid_platform_position, Vec3f &pyramid_platform_normal);

#endif