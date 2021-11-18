#include "Constants.hpp"
#include "SurfaceUtils.hpp"
#include "WallsFloors.hpp"

int find_wall(Vec3f &position, Surface **wall, bool objects) {
	if (objects) {
		int wall_idx = find_wall(position, wall, object_walls, n_object_walls);

		if (wall_idx != -1) {
			wall_idx += n_walls;
		}

		return wall_idx;
	} else {
		return find_wall(position, wall, walls, n_walls);
	}
}

int find_wall(Vec3f &position, Surface **wall, Surface wall_set[], int n_wall_set) {
	float x = position[0];
	float y = position[1] + (bully_hitbox_height / 2);
	float z = position[2];

	int active_walls = 0;
	int wall_idx = -1;

	for (int i = 0; i < n_wall_set; ++i) {
		if (y < wall_set[i].lower_y || y > wall_set[i].upper_y || x < wall_set[i].min_x - bully_radius || x > wall_set[i].max_x + bully_radius || z < wall_set[i].min_z - bully_radius || z > wall_set[i].max_z + bully_radius) {
			continue;
		}

		float offset = wall_set[i].normal[0] * x + wall_set[i].normal[1] * y + wall_set[i].normal[2] * z + wall_set[i].origin_offset;

		if (offset < -bully_radius || offset > bully_radius) {
			continue;
		}

		float sign;
		int idx;
		float pos_coord;

		if (wall_set[i].normal[0] < -0.707 || wall_set[i].normal[0] > 0.707) {
			idx = 2;
			pos_coord = z;

			if (wall_set[i].normal[0] > 0.0f) {
				sign = -1.0f;
			}
			else {
				sign = 1.0;
			}
		}
		else {
			idx = 0;
			pos_coord = x;

			if (wall_set[i].normal[2] > 0.0f) {
				sign = 1.0f;
			}
			else {
				sign = -1.0;
			}
		}

		if (sign*((wall_set[i].vertices[0][1] - y) * (wall_set[i].vertices[1][idx] - wall_set[i].vertices[0][idx]) - (wall_set[i].vertices[0][idx] - pos_coord) * (wall_set[i].vertices[1][1] - wall_set[i].vertices[0][1])) > 0.0f) {
			continue;
		}
		if (sign*((wall_set[i].vertices[1][1] - y) * (wall_set[i].vertices[2][idx] - wall_set[i].vertices[1][idx]) - (wall_set[i].vertices[1][idx] - pos_coord) * (wall_set[i].vertices[2][1] - wall_set[i].vertices[1][1])) > 0.0f) {
			continue;
		}
		if (sign*((wall_set[i].vertices[2][1] - y) * (wall_set[i].vertices[0][idx] - wall_set[i].vertices[2][idx]) - (wall_set[i].vertices[2][idx] - pos_coord) * (wall_set[i].vertices[0][1] - wall_set[i].vertices[2][1])) > 0.0f) {
			continue;
		}

		position[0] += wall_set[i].normal[0] * (bully_radius - offset);
		position[2] += wall_set[i].normal[2] * (bully_radius - offset);

		if (active_walls == 0) {
			*wall = &wall_set[i];
			wall_idx = i;
		}

		++active_walls;
	}

	return wall_idx;
}

int find_floor(Vec3f &position, Surface **floor, float &floor_y, bool objects) {
	if (objects) {
		int floor_idx = find_floor(position, floor, floor_y, object_floors, n_object_floors);

		if (floor_idx != -1) {
			floor_idx += n_floors;
		}

		return floor_idx;
	}
	else {
		return find_floor(position, floor, floor_y, floors, n_floors);
	}
}

int find_floor(Vec3f &position, Surface **floor, float &floor_y, Surface floor_set[], int n_floor_set) {
	short x = (short)(int)position[0];
	short y = (short)(int)position[1];
	short z = (short)(int)position[2];

	int floor_idx = -1;

	for (int i = 0; i < n_floor_set; ++i) {
		if (x < floor_set[i].min_x || x > floor_set[i].max_x || z < floor_set[i].min_z || z > floor_set[i].max_z) {
			continue;
		}

		if ((floor_set[i].vertices[0][2] - z) * (floor_set[i].vertices[1][0] - floor_set[i].vertices[0][0]) - (floor_set[i].vertices[0][0] - x) * (floor_set[i].vertices[1][2] - floor_set[i].vertices[0][2]) < 0) {
			continue;
		}
		if ((floor_set[i].vertices[1][2] - z) * (floor_set[i].vertices[2][0] - floor_set[i].vertices[1][0]) - (floor_set[i].vertices[1][0] - x) * (floor_set[i].vertices[2][2] - floor_set[i].vertices[1][2]) < 0) {
			continue;
		}
		if ((floor_set[i].vertices[2][2] - z) * (floor_set[i].vertices[0][0] - floor_set[i].vertices[2][0]) - (floor_set[i].vertices[2][0] - x) * (floor_set[i].vertices[0][2] - floor_set[i].vertices[2][2]) < 0) {
			continue;
		}

		float height = -(x * floor_set[i].normal[0] + floor_set[i].normal[2] * z + floor_set[i].origin_offset) / floor_set[i].normal[1];

		if (y - (height + -78.0f) < 0.0f) {
			continue;
		}

		floor_y = height;
		*floor = &floor_set[i];
		floor_idx = i;
		break;
	}

	return floor_idx;
}

bool find_surface_in_region(Surface surfaces[], int n_surfaces, float min_x, float max_x, float min_y, float max_y, float min_z, float max_z) {
	for (int h = 0; h < n_surfaces; ++h) {
		bool inside_area = false;

		if (max_y >= surfaces[h].lower_y & min_y <= surfaces[h].upper_y & max_x >= surfaces[h].min_x & min_x <= surfaces[h].max_x & max_z >= surfaces[h].min_z & min_z <= surfaces[h].max_z) {
			double t_n; double t_d;

			for (int i = 0; i < 3; ++i) {
				t_n = min_x - surfaces[h].vertices[i][0];
				t_d = surfaces[h].vertices[(i + 1) % 3][0] - surfaces[h].vertices[i][0];

				if (t_d < 0) {
					t_n = -t_n;
					t_d = -t_d;
				}

				if (t_n >= 0 && t_n <= t_d) {
					inside_area = true;
					break;
				}

				t_n = max_x - surfaces[h].vertices[i][0];
				t_d = surfaces[h].vertices[(i + 1) % 3][0] - surfaces[h].vertices[i][0];

				if (t_d < 0) {
					t_n = -t_n;
					t_d = -t_d;
				}

				if (t_n >= 0 && t_n <= t_d) {
					inside_area = true;
					break;
				}

				t_n = min_z - surfaces[h].vertices[i][2];
				t_d = surfaces[h].vertices[(i + 1) % 3][2] - surfaces[h].vertices[i][2];

				if (t_d < 0) {
					t_n = -t_n;
					t_d = -t_d;
				}

				if (t_n >= 0 && t_n <= t_d) {
					inside_area = true;
					break;
				}

				t_n = max_z - surfaces[h].vertices[i][2];
				t_d = surfaces[h].vertices[(i + 1) % 3][2] - surfaces[h].vertices[i][2];

				if (t_d < 0) {
					t_n = -t_n;
					t_d = -t_d;
				}

				if (t_n >= 0 && t_n <= t_d) {
					inside_area = true;
					break;
				}
			}
		}

		if (inside_area) {
			return true;
		}
	}

	return false;
}