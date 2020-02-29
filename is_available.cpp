#include "is_available.h"
#include <algorithm>
#include <cassert>
using namespace std;


#ifdef DEBUG_INTERSECT
#include <cstdio>
#include <iostream>
#endif

using std::max;
using std::min;

const double EPS = 1e-8;

// ������cpp����Ӷ���������

struct Line {
	Point l, r;
	Line() {};
	Line(const Point &l_, const Point &r_) : l(l_), r(r_) {}
	Line(const Line &line) : l(line.l), r(line.r) {};

	bool is_intersect(const Line &line) const;
};

struct Vehicle {
	Point light[2];
	Line car_lines[4];

	Vehicle(const Point &light_);
	Vehicle(const Vehicle &vehicle);
	bool is_intersect(const Line &line) const;
};

// class Point

Point Point::operator+(const Point &p) const {
	return Point(x + p.x, y + p.y);
}

Point Point::operator-(const Point &p) const {
	return Point(x - p.x, y - p.y);
}

// class Line

bool Line::is_intersect(const Line &line) const {
	// д��ô��line.l�Եú�ɳ��
	const Point &a = l, &b = r, &c = line.l, &d = line.r;

	// �����ų�
	if ( !(
		interval_is_intersect(min(a.x, b.x), max(a.x, b.x), min(c.x, d.x), max(c.x, d.x))
		&&
		interval_is_intersect(min(a.y, b.y), max(a.y, b.y), min(c.y, d.y), max(c.y, d.y))
		))
		return false;

	// �Ƿ����
	double u, v, w, z;
	u = (c.x - a.x)*(b.y - a.y) - (b.x - a.x)*(c.y - a.y);
	v = (d.x - a.x)*(b.y - a.y) - (b.x - a.x)*(d.y - a.y);
	w = (a.x - c.x)*(d.y - c.y) - (d.x - c.x)*(a.y - c.y);
	z = (b.x - c.x)*(d.y - c.y) - (d.x - c.x)*(b.y - c.y);

	return (u*v <= EPS && w*z <= EPS);
}

// class Vehicle

Vehicle::Vehicle(const Point &light_) {

	light[0] = light_ + Point(0, -VEHICLE_WIDTH/2);
	light[1] = light_ + Point(0, VEHICLE_WIDTH/2);
	Point l = light_ + Point(-EPS, 0);
	
	Point	a(l + Point(0, -VEHICLE_WIDTH/2)),
		b(l + Point(0, VEHICLE_WIDTH/2)),
		c(l + Point(-VEHICLE_LENGTH, VEHICLE_WIDTH/2)),
		d(l + Point(-VEHICLE_LENGTH, -VEHICLE_WIDTH/2));

#if 0
	light[0] = light_ + Point(0, -1.25);
	light[1] = light_ + Point(0, 1.25);
	Point l = light_ + Point(-EPS, 0);
	
	Point	a(l + Point(0, -1.25)),
		b(l + Point(0, 1.25)),
		c(l + Point(-4.5, 1.25)),
		d(l + Point(-4.5, -1.25));
#endif        
	car_lines[0] = Line(a, b);
	car_lines[1] = Line(b, c);
	car_lines[2] = Line(c, d);
	car_lines[3] = Line(d, a);
}

Vehicle::Vehicle(const Vehicle &vehicle) {
	for (int i = 0; i < 4; i++)
		car_lines[i] = vehicle.car_lines[i];
	light[0] = vehicle.light[0];
	light[1] = vehicle.light[1];
}

bool Vehicle::is_intersect(const Line &line) const {
	for (auto &car_line : car_lines) {
		if (line.is_intersect(car_line))
			return true;
	}
	return false;
}

bool is_intersect(const vector<Point> &lights,
	const Point &sign,
	int which) {
	vector<Vehicle> vehicles;
	// ���ÿռ��ֹ��������
	vehicles.reserve(lights.size());
	// ��ʼ��
	for (auto &light : lights)
		// �ó��Ʋ��ͳ���ǰ���غ�
		vehicles.push_back(Vehicle(light));
	// ���Ƶ�·�������
	Line line1(vehicles[which].light[0], sign);
	Line line2(vehicles[which].light[1], sign);
	// ��һ������
	bool line1_available = true, line2_available = true;
	for (auto &vehicle : vehicles) {
		if (vehicle.is_intersect(line1)) {
			//cout << "Intersect with " << (vehicle.light[0] + Point(0, 1.25)).x
			//	<< (vehicle.light[0] + Point(0, 1.25)).y << endl;
			line1_available = false;
			break;
		}
	}
	// �ڶ�������
	for (auto &vehicle : vehicles) {
		if (vehicle.is_intersect(line2)) {
			//cout << "Intersect with " << (vehicle.light[0] + Point(0, 1.25)).x
			//	<< (vehicle.light[0] + Point(0, 1.25)).y << endl;
			line2_available = false;
			break;
		}
	}
	return !line1_available && !line2_available;
}

#if 0
bool is_intersect(const vector<Point> &lights_start, const vector<Point> &light_end, const Point &sign, int which){
    if(is_intersect(lights_start, sign, which) == false && is_intersect(lights_start, sign, which) == false){
        vector<Vehicle> vehicles_start;
        vector<Vehicle> vehicles_end;
        // ���ÿռ��ֹ��������
        vehicles_start.reserve(lights_start.size());
        vehicles_end.reserve(lights_end.size());
        // ��ʼ��
        for (auto &light_start : lights_start)
            vehicles_start.push_back(Vehicle(light_start));
        for (auto &light_end : lights_end)
            vehicles_end.push_back(Vehicle(light_end));

        Line line1_start(vehicles_start[which].light[0], sign);
        Line line2_start(vehicles_start[which].light[1], sign);
        
    }else{
        return true;
    }
}
#endif

static bool interval_is_intersect(double a, double b, double c, double d) {
	assert(a <= b);
	assert(c <= d);
	if (b <= c)
		return false;
	if (d <= a)
		return false;
	return true;
}

#ifdef DEBUG_INTERSECT
int main() {
	FILE *f = fopen("D:\\soar pku\\PassiveVLC_kuntai\\mac\\mac\\test.txt", "r");
	// a little test
	vector<Point> lights;
	double x, y;
	while (fscanf(f, "(%lf, %lf)\n", &x, &y) > 0) {
		lights.push_back(Point(x, y));
	}
	cout << is_intersect(lights, Point(110, 0), 0);
	system("pause");
	/*lights.push_back(Point(-12, 9));
	lights.push_back(Point(-8.1, 4.5));
	// should be false
	assert(is_intersect(lights, Point(0, 0), 0) == 0);
	lights.push_back(Point(-7.9, 4.5));
	// should be true
	assert(is_intersect(lights, Point(0, 0), 0) == 1);*/
}
#endif