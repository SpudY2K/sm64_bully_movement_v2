#pragma once

#include "vmath.hpp"
#include <vector>

#ifndef BULLY_PATH_H
#define BULLY_PATH_H

#define STATE_CLEAR 0
#define STATE_OOB 1
#define STATE_STEEP_FLOOR 2
#define STATE_FRICTION_FLOOR 3
#define STATE_WALL 4
#define STATE_LAVA_DEATH 5

class BullyPath {
public:
	std::vector<Vec3f> frame_positions;
	std::vector<Vec3f> intended_positions;
	std::vector<int> frame_yaws;
	std::vector<float> frame_speeds;
	std::vector<int> frame_states;

	Vec3f start_pos;
	float start_speed;
	int start_yaw;

	float current_y_speed = 0.0f;
	std::vector<int> good_frames;

	BullyPath() {}

	BullyPath(Vec3f pos, int yaw, float speed, int max_frames) {
		start_pos = pos;
		start_yaw = yaw;
		start_speed = speed;

		frame_positions.reserve(2 * max_frames);
		intended_positions.reserve(2 * max_frames);
		frame_yaws.reserve(2 * max_frames);
		frame_speeds.reserve(2 * max_frames);
		frame_states.reserve(2 * max_frames);

		frame_positions.push_back(pos);
		intended_positions.push_back(pos);
		frame_yaws.push_back(yaw);
		frame_speeds.push_back(speed);
		frame_states.push_back(STATE_CLEAR);
	}

	bool advance_frame();
	float calculate_current_dist();
	void add_frame(int yaw, float speed, Vec3f &intended_position, Vec3f &next_position, int state);
};
#endif