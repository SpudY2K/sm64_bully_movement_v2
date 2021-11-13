#pragma once

#include "Surface.hpp"
#include "vmath.hpp"

#ifndef SURFACE_UTILS_H
#define SURFACE_UTILS_H

int find_wall(Vec3f &position, Surface &wall, bool object);
int find_wall(Vec3f &position, Surface &wall, std::vector<Surface> &wall_set);
int find_floor(Vec3f &position, Surface &floor, float &floor_y, bool object);
int find_floor(Vec3f &position, Surface &floor, float &floor_y, std::vector<Surface> &floor_set);
bool find_surface_in_region(std::vector<Surface> &surfaces, float min_x, float max_x, float min_y, float max_y, float min_z, float max_z);

#endif