#include "BullyPath.hpp"
#include "Constants.hpp"
#include "PathSearch.hpp"
#include "SurfaceUtils.hpp"
#include "Trig.hpp"
#include "WallsFloors.hpp"
#include <fstream>
#include <iostream>
#include <cmath>

void output_result(BullyPath &path, float lower_speed, float upper_speed, float min_offset, float max_offset) {
	int frame = 0;

	for (int i = 1; i < path.n_frames; i++) {
		if ((path.frame_states[i] & 0xF) != STATE_WALL) {
			++frame;
			float dist = euclidean_distance(path.frame_positions[i], path.start_pos);

			if (dist >= min_offset && dist <= max_offset && path.frame_states[i] != STATE_LAVA_DEATH && path.frame_speeds[i] > 0) {
				#pragma omp critical 
				{
					out_stream << frame << ", "
						<< path.start_yaw << ", "
						<< path.frame_yaws[i] << ", "
						<< std::fixed << lower_speed << ", "
						<< std::fixed << upper_speed << ", "
						<< std::fixed << path.start_speed << ", "
						<< std::fixed << path.frame_positions[i][0] << ", "
						<< std::fixed << path.frame_positions[i][1] << ", "
						<< std::fixed << path.frame_positions[i][2] << ", "
						<< std::fixed << dist << "\n";
				}
			}
		}
	}
}

bool compare_paths(BullyPath &a, BullyPath &b, int max_frames, float max_offset) {
	int frame_count = 0;

	int max_i = fmax(a.n_frames, b.n_frames);

	for (int i = 1; i < max_i; i++) {
		if (i == a.n_frames) {
			if (!a.advance_frame()) {
				return false;
			}
		}

		if (i == b.n_frames) {
			if (!b.advance_frame()) {
				return false;
			}
		}

		if (a.frame_yaws[i] != b.frame_yaws[i] || a.frame_states[i] != b.frame_states[i]) {
			return false;
		}

		float a_dist = euclidean_distance(a.frame_positions[i], a.start_pos);
		float b_dist = euclidean_distance(b.frame_positions[i], b.start_pos);

		if ((a_dist <= max_offset) ^ (b_dist <= max_offset)) {
			return false;
		}

		if (a.frame_states[i] == STATE_OOB) {
			float min_x; float max_x; float min_z; float max_z;

			if (a.intended_positions[i][0] < b.intended_positions[i][0]) {
				min_x = a.intended_positions[i][0];
				max_x = b.intended_positions[i][0];
			}
			else {
				min_x = b.intended_positions[i][0];
				max_x = a.intended_positions[i][0];
			}

			if (a.intended_positions[i][2] < b.intended_positions[i][2]) {
				min_z = a.intended_positions[i][2];
				max_z = b.intended_positions[i][2];
			}
			else {
				min_z = b.intended_positions[i][2];
				max_z = a.intended_positions[i][2];
			}

			float min_pu_x = floorf((min_x - 8192.0f) / 65536.0f) + 1;
			float max_pu_x = ceilf((max_x + 8192.0f) / 65536.0f) - 1;
			float min_pu_z = floorf((min_z - 8192.0f) / 65536.0f) + 1;
			float max_pu_z = ceilf((max_z + 8192.0f) / 65536.0f) - 1;

			if (min_pu_x <= max_pu_x && min_pu_z <= max_pu_z) {
				return false;
			}
		}
		else {
			float a_pu_x = ceilf((a.intended_positions[i][0] + 8192.0f) / 65536.0f) - 1;
			float a_pu_z = ceilf((a.intended_positions[i][2] + 8192.0f) / 65536.0f) - 1;
			float b_pu_x = ceilf((b.intended_positions[i][0] + 8192.0f) / 65536.0f) - 1;
			float b_pu_z = ceilf((b.intended_positions[i][2] + 8192.0f) / 65536.0f) - 1;

			if (a_pu_x != b_pu_x || a_pu_z != b_pu_z) {
				return false;
			}

			if (a.frame_states[i] == STATE_CLEAR) {
				float min_x; float max_x; float min_y; float max_y; float min_z; float max_z;

				if (a.intended_positions[i][0] < b.intended_positions[i][0]) {
					min_x = fmodf(a.intended_positions[i][0] + 32768.0f, 65536.0f) - 32768.0f;
					max_x = fmodf(b.intended_positions[i][0] + 32768.0f, 65536.0f) - 32768.0f;
				}
				else {
					min_x = fmodf(b.intended_positions[i][0] + 32768.0f, 65536.0f) - 32768.0f;
					max_x = fmodf(a.intended_positions[i][0] + 32768.0f, 65536.0f) - 32768.0f;
				}

				if (a.intended_positions[i][1] < b.intended_positions[i][1]) {
					min_y = a.intended_positions[i][1];
					max_y = b.intended_positions[i][1];
				}
				else {
					min_y = b.intended_positions[i][1];
					max_y = a.intended_positions[i][1];
				}

				if (a.intended_positions[i][2] < b.intended_positions[i][2]) {
					min_z = fmodf(a.intended_positions[i][2] + 32768.0f, 65536.0f) - 32768.0f;
					max_z = fmodf(b.intended_positions[i][2] + 32768.0f, 65536.0f) - 32768.0f;
				}
				else {
					min_z = fmodf(b.intended_positions[i][2] + 32768.0f, 65536.0f) - 32768.0f;
					max_z = fmodf(a.intended_positions[i][2] + 32768.0f, 65536.0f) - 32768.0f;
				}

				if (a_pu_x == 0 && a_pu_z == 0) {
					if (find_surface_in_region(walls, n_walls, min_x, max_x, min_y, max_y, min_z, max_z)) {
						return false;
					}

					if (find_surface_in_region(object_walls, n_object_walls, min_x, max_x, min_y, max_y, min_z, max_z)) {
						return false;
					}
				}

				if (find_surface_in_region(floors, n_floors, min_x, max_x, min_y - 37, max_y + 78, min_z, max_z)) {
					return false;
				}

				if (find_surface_in_region(object_floors, n_object_floors, min_x, max_x, min_y - 37, max_y + 78, min_z, max_z)) {
					return false;
				}
			}
		}

		if ((a.frame_states[i] & 0xF) != STATE_WALL) {
			++frame_count;
		}
	}

	//if (frame_count != max_frames) {
	//	Maybe add a check to see if an intermediate position could still be in range
	//}

	return true;
}

bool trace_path(BullyPath &path, int current_frame, int max_frames, float max_offset) {
	for (int n_frames = current_frame; n_frames <= max_frames; ++n_frames) {
		if (path.advance_frame()) {
			float current_dist = path.calculate_current_dist();

			if (current_dist <= max_offset && path.frame_states[path.n_frames - 1] != STATE_LAVA_DEATH && path.frame_speeds[path.n_frames - 1] > 0) {
				path.good_frames[path.n_good_frames] = n_frames;
				path.n_good_frames++;
			}

			if (current_dist - path.frame_speeds[path.n_frames - 1]*(max_frames - n_frames) > max_offset + 200 || path.frame_speeds[path.n_frames - 1] == 0) {
				break;
			}
		}
		else {
			return false;
		}
	}

	return true;
}

void search_paths(Vec3f &start_position, int max_frames, float min_offset, float max_offset, BullyPath &min_path, BullyPath &max_path, float &lower_speed_max, float &upper_speed_min) {
	if (!compare_paths(min_path, max_path, max_frames, max_offset)) {
		float min_speed = min_path.start_speed;
		float max_speed = max_path.start_speed;
		int angle = min_path.start_yaw;

		float mid_speed = (float)(((double)min_speed + (double)max_speed) / 2.0);

		if (mid_speed != min_speed && mid_speed != max_speed) {
			BullyPath mid_path(start_position, angle, mid_speed);
			trace_path(mid_path, 1, max_frames, max_offset);

			float upper_lower_speed_max = NAN;
			float upper_upper_speed_min = NAN;

			search_paths(start_position, max_frames, min_offset, max_offset, min_path, mid_path, lower_speed_max, upper_speed_min);
			search_paths(start_position, max_frames, min_offset, max_offset, mid_path, max_path, upper_lower_speed_max, upper_upper_speed_min);

			if (upper_upper_speed_min != mid_speed) {
				if (lower_speed_max != mid_speed) {
					if (!isnan(upper_speed_min)) {
						output_result(mid_path, upper_speed_min, upper_lower_speed_max, min_offset, max_offset);
					}
				}

				upper_speed_min = upper_upper_speed_min;
			}

			if (lower_speed_max == mid_speed) {
				lower_speed_max = upper_lower_speed_max;
			}
		}
		else {
			if (min_path.n_good_frames > 0) {
				if (max_path.n_good_frames > 0) {
					bool good_frames_match = (min_path.n_good_frames == max_path.n_good_frames);

					if (good_frames_match) {
						for (int i = 0; i < min_path.n_good_frames; i++) {
							if (min_path.good_frames[i] != max_path.good_frames[i]) {
								good_frames_match = false;
								break;
							}
						}
					}

					if (good_frames_match) {
						lower_speed_max = max_path.start_speed;
						upper_speed_min = min_path.start_speed;
					}
					else {
						lower_speed_max = min_path.start_speed;
						upper_speed_min = max_path.start_speed;
					}
				}
				else {
					lower_speed_max = min_path.start_speed;
				}
			}
			else if (max_path.n_good_frames > 0) {
				upper_speed_min = max_path.start_speed;
			}
		}
	}
	else {
		if (min_path.n_good_frames > 0) {
			lower_speed_max = max_path.start_speed;
			upper_speed_min = min_path.start_speed;
		}
	}
}

void find_angle_paths(Vec3f &start_position, int angle, int max_frames, float min_offset, float max_offset, float min_speed, float max_speed) {
	BullyPath min_path(start_position, angle, min_speed);
	trace_path(min_path, 1, max_frames, max_offset);

	BullyPath max_path(start_position, angle, max_speed);
	trace_path(max_path, 1, max_frames, max_offset);

	float lower_speed_max = NAN;
	float upper_speed_min = NAN;

	search_paths(start_position, max_frames, min_offset, max_offset, min_path, max_path, lower_speed_max, upper_speed_min);

	if (!isnan(lower_speed_max)) {
		output_result(min_path, min_speed, lower_speed_max, min_offset, max_offset);
	}

	if (!isnan(upper_speed_min) && upper_speed_min != min_speed) {
		output_result(min_path, upper_speed_min, max_speed, min_offset, max_offset);
	}
}

void find_paths(Vec3f &start_position, float min_speed, float max_speed, int total_frames, float min_offset, float max_offset) {
	#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < 8192; ++i) {
		std::cout << gArctanTable[i] << "\n";
		find_angle_paths(start_position, gArctanTable[i], total_frames, min_offset, max_offset, min_speed, max_speed);
	}
}