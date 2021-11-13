#pragma once

#include "BullyPath.hpp"
#include "Surface.hpp"
#include "vmath.hpp"
#include <list>

#ifndef PATH_SEARCH_H
#define PATH_SEARCH_H

extern std::ofstream out_stream;

void output_result(BullyPath &path, float lower_speed, float upper_speed, float min_offset, float max_offset);
bool compare_paths(BullyPath &a, BullyPath &b, int max_frames, float max_offset);
bool trace_path(BullyPath &path, int current_frames, int max_frames, float max_offset);
void search_paths(Vec3f &start_position, int max_frames, float min_offset, float max_offset, BullyPath &min_path, BullyPath &max_path, float &lower_speed_max, float &upper_speed_min);
void find_angle_paths(Vec3f &start_position, int angle, int max_frames, float min_offset, float max_offset, float min_speed, float max_speed);
void find_paths(Vec3f &start_position, float min_speed, float max_speed, int total_frames, float min_offset, float max_offset);

#endif