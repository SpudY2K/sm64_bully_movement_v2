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

#define MAX_FRAME_STEPS 100

class BullyPath {
public:
	Vec3f frame_positions[MAX_FRAME_STEPS];
	Vec3f intended_positions[MAX_FRAME_STEPS];
	int frame_yaws[MAX_FRAME_STEPS];
	float frame_speeds[MAX_FRAME_STEPS];
	int frame_states[MAX_FRAME_STEPS];
	int good_frames[MAX_FRAME_STEPS];

	int n_frames = 1;
	int n_good_frames = 0;

	Vec3f start_pos;
	float start_speed;
	int start_yaw;

	float current_y_speed = 0.0f;

	BullyPath() {}

	BullyPath(Vec3f pos, int yaw, float speed) {
		start_pos = pos;
		start_yaw = yaw;
		start_speed = speed;

		frame_positions[0] = pos;
		intended_positions[0] = pos;
		frame_yaws[0] = yaw;
		frame_speeds[0] = speed;
		frame_states[0] = STATE_CLEAR;
	}

	bool advance_frame();
	float calculate_current_dist();
};
#endif