#include "BullyPath.hpp"
#include "Constants.hpp"
#include "PathSearch.hpp"
#include "WallsFloors.hpp"
#include <algorithm>
#include <assert.h>
#include <cstring>
#include <fstream>
#include <string>

std::ofstream out_stream;

int main(int argc, char *argv[])
{
	int total_frames = 26;
	float min_speed = 15000000.0f;
	float max_speed = 1000000000.0f;
	float min_offset = 100.0f;
	float max_offset = 1000.0f;

	int min_angle_idx = 0;
	int max_angle_idx = 8191;

	Vec3f start_position = { -1700.0f, -2950.0f, -350.0f };
	float start_y_speed = 0.0f;

	bool pyramid_platform = true;
	bool track_platform = true;

	Vec3f track_platform_position = { -1594, -3072, 0 };
	Vec3f pyramid_platform_position = { -1945, -3225, -715 };
	Vec3f pyramid_platform_normal = { 0, 1, 0};

	std::string out_file = "BullyPositions.txt";

	bool verbose = false;

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
			printf("BitFS Bully Movement Brute Forcer.\n");
			printf("This program accepts the following options:\n\n");
			printf("-f <frames>: Maximum frames of movement considered.\n");
			printf("             Must be less than internal maximum of %d.\n", MAX_FRAME_STEPS/2);
			printf("             Default: %d\n", total_frames);
			printf("-s <min_speed> <max_speed>: Inclusive range of speeds to consider.\n");
			printf("                            Default: %g %g\n", min_speed, max_speed);
			printf("-d <min_dist> <max_dist>: Inclusive range of allowed distances from starting position.\n");
			printf("                          Routes with distance outside of this range will be discarded.\n");
			printf("                          Default: %g %g\n", min_offset, max_offset);
			printf("-a <min_idx> <max_idx>: Inclusive range of angles in arctan table to consider.\n");
			printf("                        Out of range indices will be clipped.\n");
			printf("                        Default: %d %d\n", min_angle_idx, max_angle_idx);
			printf("-p <x> <y> <z>: Starting position of the bully.\n");
			printf("                Default: %g %g %g\n", start_position[0], start_position[1], start_position[2]);
			printf("-y <spd> : Starting y speed of the bully.\n");
			printf("           Default: %g\n", start_y_speed);
			printf("-q <x> <y> <z>: Position of the track platform.\n");
			printf("                Note: the platform does not move from this position in this simulation.\n");
			printf("                Default: %g %g %g\n", track_platform_position[0], track_platform_position[1], track_platform_position[2]);
			printf("-r <x> <y> <z>: Position of the pyramid platform.\n");
			printf("                Default: %g %g %g\n", pyramid_platform_position[0], pyramid_platform_position[1], pyramid_platform_position[2]);
			printf("-n <nx> <ny> <nz>: Normal of the pyramid platform.\n");
			printf("                   Note: the platform does not move from this normal in this simulation.\n");
			printf("                   Default: %g %g %g\n", pyramid_platform_normal[0], pyramid_platform_normal[1], pyramid_platform_normal[2]);
			printf("-t: Remove the track platform from the simulation.\n");
			printf("    If provided, values of -q will be ignored.\n"); 
			printf("    Default: off\n");
			printf("-u: Remove the pyramid platform from the simulation.\n"); 
			printf("    If provided, values of -r and -s will be ignored.\n");
			printf("    Default: off\n");
			printf("-o: Path to the output CSV file of solutions.\n");
			printf("    Default: %s\n", out_file.c_str());
			printf("-v: Verbose mode. Prints all parameters used in brute force.\n");
			printf("    Default: off\n");
			printf("-h --help: Prints this text.\n");
			exit(0);
		}
		else if (!strcmp(argv[i], "-f")) {
			total_frames = std::stoi(argv[i + 1]);

			if (2 * total_frames > MAX_FRAME_STEPS) {
				printf("Error: input max frames %d is greater than the internal maximum of %d.", total_frames, MAX_FRAME_STEPS / 2);
				exit(1);
			}

			i += 1;
		}
		else if (!strcmp(argv[i], "-s")) {
			min_speed = std::stof(argv[i + 1]);
			max_speed = std::stof(argv[i + 2]);
			i += 2;
		}
		else if (!strcmp(argv[i], "-d")) {
			min_offset = std::stof(argv[i + 1]);
			max_offset = std::stof(argv[i + 2]);
			i += 2;
		}
		else if (!strcmp(argv[i], "-a")) {
			min_angle_idx = std::max(0, std::stoi(argv[i + 1]));
			max_angle_idx = std::min(8191, std::stoi(argv[i + 2]));
			i += 2;
		}
		else if (!strcmp(argv[i], "-p")) {
			start_position[0] = std::stof(argv[i + 1]);
			start_position[1] = std::stof(argv[i + 2]);
			start_position[2] = std::stof(argv[i + 3]);
			i += 3;
		}
		else if (!strcmp(argv[i], "-y")) {
			start_y_speed = std::stof(argv[i + 1]);
			i += 1;
		}
		else if (!strcmp(argv[i], "-q")) {
			track_platform_position[0] = std::stof(argv[i + 1]);
			track_platform_position[1] = std::stof(argv[i + 2]);
			track_platform_position[2] = std::stof(argv[i + 3]);
			i += 3;
		}
		else if (!strcmp(argv[i], "-r")) {
			pyramid_platform_position[0] = std::stof(argv[i + 1]);
			pyramid_platform_position[1] = std::stof(argv[i + 2]);
			pyramid_platform_position[2] = std::stof(argv[i + 3]);
			i += 3;
		}
		else if (!strcmp(argv[i], "-n")) {
			pyramid_platform_normal[0] = std::stof(argv[i + 1]);
			pyramid_platform_normal[1] = std::stof(argv[i + 2]);
			pyramid_platform_normal[2] = std::stof(argv[i + 3]);
			i += 3;
		}
		else if (!strcmp(argv[i], "-t")) {
			track_platform = false;
		}
		else if (!strcmp(argv[i], "-u")) {
			pyramid_platform = false;
		}
		else if (!strcmp(argv[i], "-o")) {
			out_file = argv[i + 1];
			i += 1;
		}
		else if (!strcmp(argv[i], "-v")) {
			verbose = true;
		}
	}

	if (verbose) {
		printf("Max Frames: %d\n", total_frames);
		printf("Speed Range: (%g, %g)\n", min_speed, max_speed);
		printf("Distance Range: (%g, %g)\n", min_offset, max_offset);
		printf("Angle Range: (%d, %d)\n", min_angle_idx, max_angle_idx);
		printf("Bully Starting Position: (%g, %g, %g)\n", start_position[0], start_position[1], start_position[2]);
		printf("Bully Starting Y Speed: %g\n", start_y_speed);

		if (pyramid_platform) {
			printf("Pyramid Platform Position: (%g, %g, %g)\n", pyramid_platform_position[0], pyramid_platform_position[1], pyramid_platform_position[2]);
			printf("Pyramid Platform Normal: (%g, %g, %g)\n", pyramid_platform_normal[0], pyramid_platform_normal[1], pyramid_platform_normal[2]);
		}
		else {
			printf("Pyramid Platform Disabled\n");
		}

		if (track_platform) {
			printf("Track Platform Position: (%g, %g, %g)\n", track_platform_position[0], track_platform_position[1], track_platform_position[2]);
		}
		else {
			printf("Track Platform Disabled\n");
		}

		printf("\n");
	}

	filter_walls_and_floors(start_position[1]);

	if (pyramid_platform) {
		add_pyramid_platform(pyramid_platform_position, pyramid_platform_normal);
	}

	if (track_platform) {
		add_track_platform(track_platform_position);
	}

	out_stream.open(out_file);

	find_paths(start_position, min_speed, max_speed, total_frames, min_offset, max_offset, min_angle_idx, max_angle_idx, start_y_speed);

	out_stream.close();
}