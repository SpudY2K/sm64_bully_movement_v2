#pragma once

#include "Surface.hpp"
#include "vmath.hpp"
#include <vector>

#ifndef WALLS_FLOORS_H
#define WALLS_FLOORS_H

extern Surface walls[];
extern Surface floors[];
extern Surface object_walls[];
extern Surface object_floors[];
extern Surface pyramid_platform_floors[];
extern Surface track_platform_floors[];
extern Surface track_platform_walls[];

extern const int total_walls;
extern const int total_floors;
extern const int total_pyramid_floors;
extern const int total_track_floors;
extern const int total_track_walls;
extern const int total_object_walls;
extern const int total_object_floors;

extern int n_walls;
extern int n_floors;
extern int n_object_floors;
extern int n_object_walls;

void filter_walls_and_floors(float y_pos);
void add_track_platform(Vec3f &track_platform_position);
void add_pyramid_platform(Vec3f &pyramid_platform_position, Vec3f &pyramid_platform_normal);

#endif