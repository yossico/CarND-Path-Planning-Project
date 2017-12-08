#ifndef PLANNER_H
#define PLANNER_H

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include <vector>

#include "defs.h"
#include "vehicle.h"
#include "road.h"

using namespace std;

class Planner {

public:

	int n;
	STATE mystate;
	vector<double> start_s;
	vector<double> end_s;
	vector<double> start_d;
	vector<double> end_d;
	LANE newlane;
	bool reducespeed;


	Planner();
	~Planner() {};

	vector<double> start_s;
	vector<double> end_s;
	vector<double> start_d;
	vector<double> end_d;

	vector<double> JMT(vector<double> start, vector <double> end, double T);
	bool safe_lane(vector<Vehicle> carsinlane , double car_s);
	void DecideState(Road myRoad, double lane, Vehicle& car);
	void ApplyState();
	LANE getLANE(double lane);

	/* FSM  transitions */
	void apply_action(Vehicle& car, LANE current_lane, LANE target_lane);
	void stay_in_lane(Vehicle& car);
	void reduce_speed(Vehicle& car);
	void change_lane(Vehicle& car, LANE target_lane);

};

#endif /* PLANNER_H */
