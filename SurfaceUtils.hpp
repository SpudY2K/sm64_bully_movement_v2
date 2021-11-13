#pragma once

#include "Surface.hpp"
#include "vmath.hpp"

#ifndef SURFACE_UTILS_H
#define SURFACE_UTILS_H

int find_wall(Vec3f &position, Surface &wall, bool object);
int find_wall(Vec3f &position, Surface &wall, std::vector<Surface> &wall_set);
int find_floor(Vec3f &position, Surface &floor, float &floor_y, bool object);
int find_floor(Vec3f &position, Surface &floor, float &floor_y, std::vector<Surface> &floor_set);

#endif