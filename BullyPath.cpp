#include "Constants.hpp"
#include "BullyPath.hpp"
#include "Surface.hpp"
#include "SurfaceUtils.hpp"
#include "Trig.hpp"
#include <climits>
#include <cmath>

float BullyPath::calculate_current_dist() {
	return euclidean_distance(start_pos, frame_positions[n_frames - 1]);
}

bool BullyPath::advance_frame() {
	if (frame_states[n_frames - 1] == STATE_LAVA_DEATH) {
		frame_yaws[n_frames] = frame_yaws[n_frames - 1];
		frame_speeds[n_frames] = frame_speeds[n_frames - 1];
		intended_positions[n_frames][0] = frame_positions[n_frames - 1][0];
		intended_positions[n_frames][1] = frame_positions[n_frames - 1][1] - 10;
		intended_positions[n_frames][2] = frame_positions[n_frames - 1][2];
		frame_positions[n_frames][0] = frame_positions[n_frames - 1][0];
		frame_positions[n_frames][1] = frame_positions[n_frames - 1][1] - 10;
		frame_positions[n_frames][2] = frame_positions[n_frames - 1][2];
		frame_states[n_frames] = STATE_LAVA_DEATH;
	}
	else {
		int current_hau = (uint16_t)frame_yaws[n_frames - 1] >> 4;

		float x_speed = frame_speeds[n_frames - 1] * gSineTable[current_hau];
		float z_speed = frame_speeds[n_frames - 1] * gCosineTable[current_hau];

		intended_positions[n_frames][0] = frame_positions[n_frames - 1][0] + x_speed;
		intended_positions[n_frames][1] = frame_positions[n_frames - 1][1];
		intended_positions[n_frames][2] = frame_positions[n_frames - 1][2] + z_speed;

		if (intended_positions[n_frames][0] < INT_MIN || intended_positions[n_frames][0] > INT_MAX || intended_positions[n_frames][2] < INT_MIN || intended_positions[n_frames][2] > INT_MAX) {
			return false;
		}

		if (intended_positions[n_frames][0] > -8192 && intended_positions[n_frames][0] < 8192 && intended_positions[n_frames][2] > -8192 && intended_positions[n_frames][2] < 8192) {
			frame_positions[n_frames][0] = intended_positions[n_frames][0];
			frame_positions[n_frames][1] = intended_positions[n_frames][1];
			frame_positions[n_frames][2] = intended_positions[n_frames][2];

			Surface *wall;
			int wall_idx = find_wall(frame_positions[n_frames], &wall, true);

			if (wall_idx == -1) {
				wall_idx = find_wall(frame_positions[n_frames], &wall, false);
			}
			else {
				Surface *object_wall;
				find_wall(frame_positions[n_frames], &object_wall, false);
			}

			if (wall_idx != -1) {
				float wall_normal_norm1 = wall->normal[0] * wall->normal[0] + wall->normal[2] * wall->normal[2];
				float nx_nz_sq_diff = wall->normal[0] * wall->normal[0] - wall->normal[2] * wall->normal[2];
				float nx_nz = wall->normal[0] * wall->normal[2];

				float yaw_x = -nx_nz_sq_diff * x_speed / wall_normal_norm1 - 2 * z_speed * nx_nz / wall_normal_norm1;
				float yaw_z = nx_nz_sq_diff * z_speed / wall_normal_norm1 - 2 * x_speed * nx_nz / wall_normal_norm1;

				frame_yaws[n_frames] = atan2s(yaw_z, yaw_x);
				current_hau = (uint16_t)frame_yaws[n_frames] >> 4;
				frame_speeds[n_frames] = frame_speeds[n_frames - 1];
				frame_states[n_frames] = (wall_idx << 4) | STATE_WALL;
				n_frames++;
				intended_positions[n_frames][0] = intended_positions[n_frames - 1][0];
				intended_positions[n_frames][1] = intended_positions[n_frames - 1][1];
				intended_positions[n_frames][2] = intended_positions[n_frames - 1][2];
			}
		}

		short x_mod = (short)(int)intended_positions[n_frames][0];
		short z_mod = (short)(int)intended_positions[n_frames][2];

		if (abs(x_mod) < 8192 && abs(z_mod) < 8192) {
			Surface *floor;
			float floorY = lava_y;
			int floor_idx = find_floor(intended_positions[n_frames], &floor, floorY, false);

			Surface *object_floor;
			float object_floorY = lava_y;
			int object_floor_idx = find_floor(intended_positions[n_frames], &object_floor, object_floorY, true);

			if (floor_idx == -1) {
				if (object_floor_idx != -1) {
					floor = object_floor;
					floorY = object_floorY;
					floor_idx = object_floor_idx;
				}
			}
			else if (object_floor_idx != -1) {
				if (object_floorY > floorY) {
					floor = object_floor;
					floorY = object_floorY;
					floor_idx = object_floor_idx;
				}
			}

			if (floor_idx != -1) {
				if (floor->normal[1] < 0.5 && floorY > intended_positions[n_frames][1]) {
					float floor_normal_norm1 = floor->normal[0] * floor->normal[0] + floor->normal[2] * floor->normal[2];
					float nx_nz_sq_diff = floor->normal[0] * floor->normal[0] - floor->normal[2] * floor->normal[2];
					float nx_nz = floor->normal[0] * floor->normal[2];

					float yaw_x = -nx_nz_sq_diff * x_speed / floor_normal_norm1 - 2 * z_speed * nx_nz / floor_normal_norm1;
					float yaw_z = nx_nz_sq_diff * z_speed / floor_normal_norm1 - 2 * x_speed * nx_nz / floor_normal_norm1;

					frame_yaws[n_frames] = atan2s(yaw_z, yaw_x);
					int next_hau = (uint16_t)frame_yaws[n_frames] >> 4;

					frame_positions[n_frames][0] = frame_positions[n_frames - 1][0] + frame_speeds[n_frames - 1] * gSineTable[next_hau];
					frame_positions[n_frames][1] = frame_positions[n_frames - 1][1];
					frame_positions[n_frames][2] = frame_positions[n_frames - 1][2] + frame_speeds[n_frames - 1] * gCosineTable[next_hau];

					if (frame_positions[n_frames][0] < INT_MIN || frame_positions[n_frames][0] > INT_MAX || frame_positions[n_frames][2] < INT_MIN || frame_positions[n_frames][2] > INT_MAX) {
						return false;
					}

					frame_speeds[n_frames] = frame_speeds[n_frames - 1];
					frame_states[n_frames] = (floor_idx << 4) | STATE_STEEP_FLOOR;
				}
				else {
					current_y_speed -= bully_gravity;

					if (current_y_speed < -75.0) {
						current_y_speed = -75.0;
					}

					frame_positions[n_frames][1] = frame_positions[n_frames - 1][1] + current_y_speed;

					if (frame_positions[n_frames][1] < floorY) {
						frame_positions[n_frames][1] = floorY;

						if (current_y_speed < -17.5) {
							current_y_speed = -(current_y_speed / 2);
						}
						else {
							current_y_speed = 0;
						}
					}

					if ((int)frame_positions[n_frames][1] >= (int)floorY && (int)frame_positions[n_frames][1] < (int)floorY + 37) {
						float floor_normal_norm1 = floor->normal[0] * floor->normal[0] + floor->normal[2] * floor->normal[2];
						float floor_normal_norm2 = floor->normal[0] * floor->normal[0] + floor->normal[1] * floor->normal[1] + floor->normal[2] * floor->normal[2];

						x_speed += floor->normal[0] * floor_normal_norm1 / floor_normal_norm2 * bully_gravity * 2;
						z_speed += floor->normal[2] * floor_normal_norm1 / floor_normal_norm2 * bully_gravity * 2;

						frame_yaws[n_frames] = atan2s(z_speed, x_speed);
						int next_hau = (uint16_t)frame_yaws[n_frames] >> 4;

						if (floor->normal[1] < 0.2) {
							frame_speeds[n_frames] = 0.0f;
						}
						else {
              // equivalent to sqrtf(x_speed * x_speed + z_speed * z_speed)
							frame_speeds[n_frames] = hypotf(x_speed, z_speed) * bully_friction;
						}

						frame_positions[n_frames][0] = frame_positions[n_frames - 1][0] + frame_speeds[n_frames] * gSineTable[next_hau];
						frame_positions[n_frames][2] = frame_positions[n_frames - 1][2] + frame_speeds[n_frames] * gCosineTable[next_hau];

						if (frame_positions[n_frames][0] < INT_MIN || frame_positions[n_frames][0] > INT_MAX || frame_positions[n_frames][2] < INT_MIN || frame_positions[n_frames][2] > INT_MAX) {
							return false;
						}

						if (frame_positions[n_frames][1] == lava_y) {
							frame_states[n_frames] = STATE_LAVA_DEATH;
						}
						else {
							frame_states[n_frames] = (floor_idx << 4) | STATE_FRICTION_FLOOR;
						}
					}
					else {
						frame_yaws[n_frames] = frame_yaws[n_frames - 1];
						frame_speeds[n_frames] = frame_speeds[n_frames - 1];
						frame_positions[n_frames][0] = frame_positions[n_frames - 1][0] + frame_speeds[n_frames] * gSineTable[current_hau];
						frame_positions[n_frames][2] = frame_positions[n_frames - 1][2] + frame_speeds[n_frames] * gCosineTable[current_hau];
						frame_states[n_frames] = STATE_CLEAR;
					}
				}
			}
			else {
				current_y_speed -= bully_gravity;

				if (current_y_speed < -75.0) {
					current_y_speed = -75.0;
				}

				frame_positions[n_frames][1] = frame_positions[n_frames - 1][1] + current_y_speed;

				if (frame_positions[n_frames][1] < floorY) {
					frame_positions[n_frames][1] = floorY;

					if (current_y_speed < -17.5) {
						current_y_speed = -(current_y_speed / 2);
					}
					else {
						current_y_speed = 0;
					}
				}

				frame_yaws[n_frames] = frame_yaws[n_frames - 1];
				frame_speeds[n_frames] = frame_speeds[n_frames - 1];
				frame_positions[n_frames][0] = frame_positions[n_frames - 1][0] + frame_speeds[n_frames] * gSineTable[current_hau];
				frame_positions[n_frames][2] = frame_positions[n_frames - 1][2] + frame_speeds[n_frames] * gCosineTable[current_hau];
				frame_states[n_frames] = STATE_CLEAR;
			}
		}
		else {
			frame_yaws[n_frames] = frame_yaws[n_frames - 1] + 32767.999200000002;
			int next_hau = (uint16_t)frame_yaws[n_frames] >> 4;
			frame_speeds[n_frames] = frame_speeds[n_frames - 1];
			frame_positions[n_frames][0] = frame_positions[n_frames - 1][0] + frame_speeds[n_frames] * gSineTable[next_hau];
			frame_positions[n_frames][1] = frame_positions[n_frames - 1][1];
			frame_positions[n_frames][2] = frame_positions[n_frames - 1][2] + frame_speeds[n_frames] * gCosineTable[next_hau];

			if (frame_positions[n_frames][0] < INT_MIN || frame_positions[n_frames][0] > INT_MAX || frame_positions[n_frames][2] < INT_MIN || frame_positions[n_frames][2] > INT_MAX) {
				return false;
			}

			frame_states[n_frames] = STATE_OOB;
		}
	}

	n_frames++;

	return true;
}