#include "Constants.hpp"
#include "SurfaceUtils.hpp"
#include "WallsFloors.hpp"

int find_wall(Vec3f &position, Surface &wall, bool objects) {
	if (objects) {
		int wall_idx = find_wall(position, wall, object_walls);

		if (wall_idx != -1) {
			wall_idx += walls.size();
		}

		return wall_idx;
	} else {
		return find_wall(position, wall, walls);
	}
}

int find_wall(Vec3f &position, Surface &wall, std::vector<Surface> &wall_set) {
	float x = position[0];
	float y = position[1] + (bully_hitbox_height / 2);
	float z = position[2];

	int active_walls = 0;
	int wall_count = -1;
	int wall_idx = -1;

	for (Surface w : wall_set) {
		++wall_count;

		if (y < w.lower_y || y > w.upper_y || x < w.min_x - bully_radius || x > w.max_x + bully_radius || z < w.min_z - bully_radius || z > w.max_z + bully_radius) {
			continue;
		}

		float offset = w.normal[0] * x + w.normal[1] * y + w.normal[2] * z + w.origin_offset;

		if (offset < -bully_radius || offset > bully_radius) {
			continue;
		}

		float sign;
		int idx;
		float pos_coord;

		if (w.normal[0] < -0.707 || w.normal[0] > 0.707) {
			idx = 2;
			pos_coord = z;

			if (w.normal[0] > 0.0f) {
				sign = -1.0f;
			}
			else {
				sign = 1.0;
			}
		}
		else {
			idx = 0;
			pos_coord = x;

			if (w.normal[2] > 0.0f) {
				sign = 1.0f;
			}
			else {
				sign = -1.0;
			}
		}

		if (sign*((w.vertices[0][1] - y) * (w.vertices[1][idx] - w.vertices[0][idx]) - (w.vertices[0][idx] - pos_coord) * (w.vertices[1][1] - w.vertices[0][1])) > 0.0f) {
			continue;
		}
		if (sign*((w.vertices[1][1] - y) * (w.vertices[2][idx] - w.vertices[1][idx]) - (w.vertices[1][idx] - pos_coord) * (w.vertices[2][1] - w.vertices[1][1])) > 0.0f) {
			continue;
		}
		if (sign*((w.vertices[2][1] - y) * (w.vertices[0][idx] - w.vertices[2][idx]) - (w.vertices[2][idx] - pos_coord) * (w.vertices[0][1] - w.vertices[2][1])) > 0.0f) {
			continue;
		}

		position[0] += w.normal[0] * (bully_radius - offset);
		position[2] += w.normal[2] * (bully_radius - offset);

		if (active_walls == 0) {
			wall = w;
			wall_idx = wall_count;
		}

		++active_walls;
	}

	return wall_idx;
}

int find_floor(Vec3f &position, Surface &floor, float &floor_y, bool objects) {
	if (objects) {
		int floor_idx = find_floor(position, floor, floor_y, object_floors);

		if (floor_idx != -1) {
			floor_idx += floors.size();
		}

		return floor_idx;
	}
	else {
		return find_floor(position, floor, floor_y, floors);
	}
}

int find_floor(Vec3f &position, Surface &floor, float &floor_y, std::vector<Surface> &floor_set) {
	short x = (short)(int)position[0];
	short y = (short)(int)position[1];
	short z = (short)(int)position[2];

	int floor_idx = -1;
	int floor_count = -1;

	for (Surface f : floor_set) {
		++floor_count;

		if (x < f.min_x || x > f.max_x || z < f.min_z || z > f.max_z) {
			continue;
		}

		if ((f.vertices[0][2] - z) * (f.vertices[1][0] - f.vertices[0][0]) - (f.vertices[0][0] - x) * (f.vertices[1][2] - f.vertices[0][2]) < 0) {
			continue;
		}
		if ((f.vertices[1][2] - z) * (f.vertices[2][0] - f.vertices[1][0]) - (f.vertices[1][0] - x) * (f.vertices[2][2] - f.vertices[1][2]) < 0) {
			continue;
		}
		if ((f.vertices[2][2] - z) * (f.vertices[0][0] - f.vertices[2][0]) - (f.vertices[2][0] - x) * (f.vertices[0][2] - f.vertices[2][2]) < 0) {
			continue;
		}

		float height = -(x * f.normal[0] + f.normal[2] * z + f.origin_offset) / f.normal[1];

		if (y - (height + -78.0f) < 0.0f) {
			continue;
		}

		floor_y = height;
		floor = f;
		floor_idx = floor_count;
		break;
	}

	return floor_idx;
}

bool find_surface_in_region(std::vector<Surface> &surfaces, float min_x, float max_x, float min_y, float max_y, float min_z, float max_z) {
	for (Surface s : surfaces) {
		bool inside_area = false;

		if (max_y >= s.lower_y && min_y <= s.upper_y && max_x >= s.min_x && min_x <= s.max_x && max_z >= s.min_z && min_z <= s.max_z) {
			double t_n; double t_d;

			for (int i = 0; i < 3; ++i) {
				t_n = min_x - s.vertices[i][0];
				t_d = s.vertices[(i + 1) % 3][0] - s.vertices[i][0];

				if (t_d < 0) {
					t_n = -t_n;
					t_d = -t_d;
				}

				if (t_n >= 0 && t_n <= t_d) {
					inside_area = true;
					break;
				}

				t_n = max_x - s.vertices[i][0];
				t_d = s.vertices[(i + 1) % 3][0] - s.vertices[i][0];

				if (t_d < 0) {
					t_n = -t_n;
					t_d = -t_d;
				}

				if (t_n >= 0 && t_n <= t_d) {
					inside_area = true;
					break;
				}

				t_n = min_z - s.vertices[i][2];
				t_d = s.vertices[(i + 1) % 3][2] - s.vertices[i][2];

				if (t_d < 0) {
					t_n = -t_n;
					t_d = -t_d;
				}

				if (t_n >= 0 && t_n <= t_d) {
					inside_area = true;
					break;
				}

				t_n = max_z - s.vertices[i][2];
				t_d = s.vertices[(i + 1) % 3][2] - s.vertices[i][2];

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