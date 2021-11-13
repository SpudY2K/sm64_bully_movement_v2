#include "Constants.hpp"
#include "PathSearch.hpp"
#include "WallsFloors.hpp"
#include <fstream>
#include <iostream>

std::ofstream out_stream;

int main()
{
	int total_frames = 26;
	float min_speed = 15000000.0f;
	float max_speed = 1000000000.0f;
	float min_offset = 100.0f;
	float max_offset = 1000.0f;

	Vec3f start_position = { -1700.0f, -2950.0f, -350.0f };

	bool pyramid_platform = true;
	bool track_platform = true;

	Vec3f track_platform_position = { -1594, -3072, 0 };
	Vec3f pyramid_platform_position = { -1945, -3225, -715 };
	Vec3f pyramid_platform_normal = { 0, 1, 0};

	filter_walls_and_floors(start_position[1]);

	if (pyramid_platform) {
		add_pyramid_platform(pyramid_platform_position, pyramid_platform_normal);
	}

	if (track_platform) {
		add_track_platform(track_platform_position);
	}

	out_stream.open("BullyPositions.txt");

	find_paths(start_position, min_speed, max_speed, total_frames, min_offset, max_offset);

	out_stream.close();
}