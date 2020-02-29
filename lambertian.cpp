

#include "lambertian.h"
#include <cmath>
#include <algorithm>
#include <iostream>
using namespace std;

#define VEHICLE_LENGTH  4.5
#define VEHICLE_WIDTH   2.5


const double pi = acos(-1), EPS = 1e-6, INF = 1e30, step = 5e-2;

static double convert(double deg) {
	return pi * deg / 180;
}

bool is_connected(double distance, double cos_angle, double max_distance, double fov) {
//	if (cos_angle < cos(convert(fov)))
	if (cos_angle < cos(fov))
		return false;
	//double m = -log(2) / log(cos(convert(fov)));
	double m = -log(2) / log(cos(fov));
	//int m = 1;	// fov = 60
	double threashold = 1 / (max_distance * max_distance);
	double intensity = 1 / (distance * distance) * pow(cos_angle, m + 1);	// reader³öÉä½Ç=readerÈëÉä½Ç
	return intensity >= threashold;
}

bool is_connected(double distance, double cos_angle, double max_distance) {
    return is_connected(distance, cos_angle, max_distance, convert(60));
}

bool is_connected(double vehicle_x, double vehicle_y, double tag_x, double tag_y, double max_distance, double fov){
    double start_delta_x = vehicle_x - tag_x;
    if(start_delta_x >= 0){
        return false;
    }
   
    double start_delta_l_y = vehicle_y + VEHICLE_WIDTH/2 - tag_y;
    double start_delta_r_y = vehicle_y - VEHICLE_WIDTH/2 - tag_y;

    double start_distance_l = sqrt(pow(start_delta_x, 2) + pow(start_delta_l_y, 2)); 
    double start_degree_l = atan(fabs(start_delta_l_y)/fabs(start_delta_x));

    double start_distance_r = sqrt(pow(start_delta_x, 2) + pow(start_delta_r_y, 2)); 
    double start_degree_r = atan(fabs(start_delta_r_y)/fabs(start_delta_x));
	return is_connected(start_distance_l, cos(start_degree_l), max_distance, fov) | is_connected(start_distance_r, cos(start_degree_r), max_distance, fov); 
}

int get_xaxis_range(double y, double tagx, double tagy, double max_distance, double &l, double &r) {
	// Sanity check
	y -= tagy;
	if (y < 0)
		return -1;
	if (y > max_distance)
		return -1;
	l = 1;
	r = -max_distance - 1;
	for (double i = -max_distance - step; i < 2 * step; i += step) {
		double hypotenuse = sqrt(y * y + i * i);
		if (is_connected(hypotenuse, -i/hypotenuse, max_distance)) {
			l = min(l, i);
			r = max(r, i);
		}
	}
	r += step;
	l += tagx;
	r += tagx;
	return l <= r;
}

#ifdef DEBUG_LAMBERTIAN
int main() {
	double l, r;
	get_xaxis_range(0, 0, 0, 80, l, r);
	cout << l << ' ' << r << endl;
	get_xaxis_range(40, 0, 0, 80, l, r);
	cout << l << ' ' << r << endl;
	get_xaxis_range(5, 0, 0, 80, l, r);
	cout << l << ' ' << r << endl;
	system("pause");
	return 0;
}

#define TAG_FOV 40.0
#define PI  3.14

#include <stdio.h>
int main(){
    if(is_connected(-7.015966, 2.750000 , 110.000000, 0.0, 120, TAG_FOV/2*PI/180)){
        printf("reader1 in range\n");
    }
    if(is_connected(102.698545, 9.750000, 235.000000, 0.0, 120, TAG_FOV/2*PI/180)){
        printf("reader2 in range\n");
    }
}
#endif
