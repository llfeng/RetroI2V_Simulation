#pragma once

#ifndef IS_AVAILABLE_H
#define IS_AVAILABLE_H
#include <vector>
using std::vector;



#define VEHICLE_LENGTH  4.5
#define VEHICLE_WIDTH   2.5


#define X get_x()
#define Y get_y()

struct Point {
	double x, y;
	Point(double x_ = 0, double y_ = 0) : x(x_), y(y_) {};
	Point(const Point &p) : x(p.x), y(p.y) {};
	Point operator+(const Point &p) const;
	Point operator-(const Point &p) const;
};

bool is_intersect(const vector<Point> &lights, 
	const Point &sign, 
	int which);

static bool interval_is_intersect(double a, double b, double c, double d);
#endif
