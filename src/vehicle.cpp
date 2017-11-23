#include "vehicle.h"

using namespace std;

Vehicle::Vehicle() {
	this->id = -1;
}

Vehicle::Vehicle(int id, double x, double y, double v, double s, double d) {
	this->id = id;
	this->x = x;
	this->y = y;
	this->v = v;
	this->s = s;
	this->d = d;
}

void Vehicle::update(double x, double y, double v, double s, double d, double yaw) {
	this->x = x;
	this->y = y;
	this->v = v;
	this->s = s;
	this->d = d;
	this->yaw = yaw;
}


/* GETTERS*/
int Vehicle::get_id() {
	return this->id;
}

double Vehicle::get_x() {
	return this->x;
}

double Vehicle::get_y() {
	return this->y;
}

double Vehicle::get_v() {
	return this->v;
}

double Vehicle::get_s() {
	return this->s;
}

double Vehicle::get_d() {
	return this->d;
}

LANE Vehicle::lane() {
	LANE lane;
	if (this->d < 4.0) {
		lane = LANE::LEFT;
	}
	else if ((this->d >= 4.0) && (this->d < 8.0)) {
		lane = LANE::CENTER;
	}
	else {
		lane = LANE::RIGHT;
	}
	return lane;
}

