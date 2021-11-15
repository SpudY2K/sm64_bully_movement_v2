#pragma once

#include "Surface.hpp"
#include "vmath.hpp"

#ifndef SURFACE_UTILS_H
#define SURFACE_UTILS_H

int find_wall(Vec3f &position, Surface **wall, bool object);
int find_wall(Vec3f &position, Surface **wall, Surface wall_set[], int n_wall_set);
int find_floor(Vec3f &position, Surface **floor, float &floor_y, bool object);
int find_floor(Vec3f &position, Surface **floor, float &floor_y, Surface floor_set[], int n_floor_set);
bool find_surface_in_region(Surface surfaces[], int n_surfaces, float min_x, float max_x, float min_y, float max_y, float min_z, float max_z);

#endif