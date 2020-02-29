#pragma once

#include <cmath>
#include <utility>
using std::pair;

bool is_connected(double light_x, double light_y, double tag_x, double tag_y, double max_distance, double fov);

//bool is_connected(double distance, double angle, double max_distance, double fov);

int get_xaxis_range(double y, double tagx, double tagy, double max_distance, double &l, double &r);
