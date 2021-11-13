#include "Constants.hpp"
#include "BullyPath.hpp"
#include "Surface.hpp"
#include "SurfaceUtils.hpp"
#include "Trig.hpp"
#include <cmath>

float BullyPath::calculate_current_dist() {
	return euclidean_distance(start_pos, frame_positions.back());
}

bool BullyPath::advance_frame() {
	Vec3f* current_position = &frame_positions.back();
	int current_state = frame_states.back();
	float current_speed = frame_speeds.back();
	int current_yaw = frame_yaws.back();

	if (current_state == STATE_LAVA_DEATH) {
		Vec3f next_position = *current_position;
		next_position[1] += -10;

		frame_yaws.push_back(current_yaw);
		frame_speeds.push_back(current_speed);
		intended_positions.push_back(next_position);
		frame_positions.push_back(next_position);
		frame_states.push_back(STATE_LAVA_DEATH);
	}
	else {
		int current_hau = (uint16_t)current_yaw >> 4;

		float x_speed = current_speed * gSineTable[current_hau];
		float z_speed = current_speed * gCosineTable[current_hau];

		Vec3f intended_position = *current_position;
		intended_position[0] += x_speed;
		intended_position[2] += z_speed;

		if (intended_position[0] < INT_MIN || intended_position[0] > INT_MAX || intended_position[2] < INT_MIN || intended_position[2] > INT_MAX) {
			return false;
		}

		if (intended_position[0] > -8192 && intended_position[0] < 8192 && intended_position[2] > -8192 && intended_position[2] < 8192) {
			Surface wall;
			int wall_idx = find_wall(intended_position, wall, true);

			if (wall_idx == -1) {
				wall_idx = find_wall(intended_position, wall, false);
			}
			else {
				Surface object_wall;
				find_wall(intended_position, object_wall, false);
			}

			if (wall_idx != -1) {
				int current_hau = (uint16_t)current_yaw >> 4;

				float wall_normal_norm1 = wall.normal[0] * wall.normal[0] + wall.normal[2] * wall.normal[2];
				float nx_nz_sq_diff = wall.normal[0] * wall.normal[0] - wall.normal[2] * wall.normal[2];
				float nx_nz = wall.normal[0] * wall.normal[2];

				float yaw_x = -nx_nz_sq_diff * x_speed / wall_normal_norm1 - 2 * z_speed * nx_nz / wall_normal_norm1;
				float yaw_z = nx_nz_sq_diff * z_speed / wall_normal_norm1 - 2 * x_speed * nx_nz / wall_normal_norm1;

				int next_yaw = atan2s(yaw_z, yaw_x);
				int next_hau = (uint16_t)next_yaw >> 4;

				frame_yaws.push_back(next_yaw);
				frame_speeds.push_back(current_speed);
				intended_positions.push_back(intended_position);
				frame_positions.push_back(intended_position);
				frame_states.push_back((wall_idx << 4) | STATE_WALL);

				current_position = &intended_position;
				current_yaw = next_yaw;
				current_hau = next_hau;
			}
		}

		short x_mod = (short)(int)intended_position[0];
		short z_mod = (short)(int)intended_position[2];

		if (abs(x_mod) < 8192 && abs(z_mod) < 8192) {
			Surface floor;
			float floorY = lava_y;
			int floor_idx = find_floor(intended_position, floor, floorY, false);

			Surface object_floor;
			float object_floorY = lava_y;
			int object_floor_idx = find_floor(intended_position, object_floor, object_floorY, true);

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
				int current_hau = (uint16_t)current_yaw >> 4;

				if (floor.normal[1] < 0.5 && floorY > intended_position[1]) {
					float floor_normal_norm1 = floor.normal[0] * floor.normal[0] + floor.normal[2] * floor.normal[2];
					float nx_nz_sq_diff = floor.normal[0] * floor.normal[0] - floor.normal[2] * floor.normal[2];
					float nx_nz = floor.normal[0] * floor.normal[2];

					float yaw_x = -nx_nz_sq_diff * x_speed / floor_normal_norm1 - 2 * z_speed * nx_nz / floor_normal_norm1;
					float yaw_z = nx_nz_sq_diff * z_speed / floor_normal_norm1 - 2 * x_speed * nx_nz / floor_normal_norm1;

					int next_yaw = atan2s(yaw_z, yaw_x);
					int next_hau = (uint16_t)next_yaw >> 4;

					Vec3f next_position = *current_position;
					next_position[0] += current_speed * gSineTable[next_hau];
					next_position[2] += current_speed * gCosineTable[next_hau];

					if (next_position[0] < INT_MIN || next_position[0] > INT_MAX || next_position[2] < INT_MIN || next_position[2] > INT_MAX) {
						return false;
					}

					frame_yaws.push_back(next_yaw);
					frame_speeds.push_back(current_speed);
					intended_positions.push_back(intended_position);
					frame_positions.push_back(next_position);
					frame_states.push_back((floor_idx << 4) | STATE_STEEP_FLOOR);
				}
				else {
					current_y_speed -= bully_gravity;

					if (current_y_speed < -75.0) {
						current_y_speed = -75.0;
					}

					float next_y = (*current_position)[1] + current_y_speed;

					if (next_y < floorY) {
						next_y = floorY;

						if (current_y_speed < -17.5) {
							current_y_speed = -(current_y_speed / 2);
						}
						else {
							current_y_speed = 0;
						}
					}

					if ((int)next_y >= (int)floorY && (int)next_y < (int)floorY + 37) {

						float floor_normal_norm1 = floor.normal[0] * floor.normal[0] + floor.normal[2] * floor.normal[2];
						float floor_normal_norm2 = floor.normal[0] * floor.normal[0] + floor.normal[1] * floor.normal[1] + floor.normal[2] * floor.normal[2];

						x_speed += floor.normal[0] * floor_normal_norm1 / floor_normal_norm2 * bully_gravity * 2;
						z_speed += floor.normal[2] * floor_normal_norm1 / floor_normal_norm2 * bully_gravity * 2;

						int next_yaw = atan2s(z_speed, x_speed);
						int next_hau = (uint16_t)next_yaw >> 4;

						float next_speed;

						if (floor.normal[1] < 0.2) {
							next_speed = 0.0f;
						}
						else {
							next_speed = sqrtf(x_speed*x_speed + z_speed * z_speed)*bully_friction;
						}

						Vec3f next_position = *current_position;
						next_position[0] += next_speed * gSineTable[next_hau];
						next_position[1] = next_y;
						next_position[2] += next_speed * gCosineTable[next_hau];

						if (next_position[0] < INT_MIN || next_position[0] > INT_MAX || next_position[2] < INT_MIN || next_position[2] > INT_MAX) {
							return false;
						}

						frame_yaws.push_back(next_yaw);
						frame_speeds.push_back(next_speed);
						intended_positions.push_back(intended_position);
						frame_positions.push_back(next_position);

						if (next_y == lava_y) {
							frame_states.push_back(STATE_LAVA_DEATH);
						}
						else {
							frame_states.push_back((floor_idx << 4) | STATE_FRICTION_FLOOR);
						}
					}
					else {
						Vec3f next_position = *current_position;
						next_position[0] += current_speed * gSineTable[current_hau];
						next_position[1] = next_y;
						next_position[2] += current_speed * gCosineTable[current_hau];

						frame_yaws.push_back(current_yaw);
						frame_speeds.push_back(current_speed);
						intended_positions.push_back(intended_position);
						frame_positions.push_back(next_position);
						frame_states.push_back(STATE_CLEAR);
					}
				}
			}
			else {
				current_y_speed -= bully_gravity;

				if (current_y_speed < -75.0) {
					current_y_speed = -75.0;
				}

				float next_y = (*current_position)[1] + current_y_speed;

				if (next_y < floorY) {
					next_y = floorY;

					if (current_y_speed < -17.5) {
						current_y_speed = -(current_y_speed / 2);
					}
					else {
						current_y_speed = 0;
					}
				}

				Vec3f next_position = *current_position;
				next_position[0] += current_speed * gSineTable[current_hau];
				next_position[1] = next_y;
				next_position[2] += current_speed * gCosineTable[current_hau];

				frame_yaws.push_back(current_yaw);
				frame_speeds.push_back(current_speed);
				intended_positions.push_back(intended_position);
				frame_positions.push_back(next_position);
				frame_states.push_back(STATE_CLEAR);
			}
		}
		else {
			int next_yaw = current_yaw + 32767.999200000002;
			int next_hau = (uint16_t)next_yaw >> 4;

			float next_speed = current_speed;

			Vec3f next_position = *current_position;
			next_position[0] += next_speed * gSineTable[next_hau];
			next_position[2] += next_speed * gCosineTable[next_hau];

			if (next_position[0] < INT_MIN || next_position[0] > INT_MAX || next_position[2] < INT_MIN || next_position[2] > INT_MAX) {
				return false;
			}

			frame_yaws.push_back(next_yaw);
			frame_speeds.push_back(next_speed);
			intended_positions.push_back(intended_position);
			frame_positions.push_back(next_position);
			frame_states.push_back(STATE_OOB);
		}
	}

	return true;
}