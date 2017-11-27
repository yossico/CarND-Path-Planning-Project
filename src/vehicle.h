#pragma once 

#include <vector>
#include "defs.h"

using namespace std;

class Vehicle {

    /* The Car variables storage and update
	id: car's unique ID,
	x: car's x position in map coordinates,
	y: car's y position in map coordinates,
	v: car's speed,
	s: car's s position in frenet coordinates,
	d: car's d position in frenet coordinates. */

public:

	int id;
	double x;
	double y;
	double v;
	double s;
	double d;
	double yaw;

	vector<double> previous_s;
	vector<double> previous_d;

	Vehicle();
	Vehicle(int id, double x, double y, double v, double s, double d);
	~Vehicle() {};


	LANE lane();
	int get_id();
	double get_x();
	double get_y();
	double get_v();
	double get_s();
	double get_d();
	double get_yaw();

	double lanenum();

	void update(double x, double y, double v, double s, double d, double yaw);
	void set_previous_s(vector<double> previous_s);
	void set_previous_d(vector<double> previous_d);
	vector<double> prev_s();
	vector<double> prev_d();

};