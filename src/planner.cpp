#include "defs.h"
#include "planner.h"

using namespace std;

Planner::Planner() {
	this->mystate = STATE::START;
}


bool Planner::safe_lane(vector<Vehicle> carsinlane, double car_s)
{
	for (Vehicle other_car : carsinlane) 
	{
		double s_diff = fabs(other_car.s - car_s);
		if (s_diff < GUARD_DISTANCE)
		{
			return false;
		}
	}
	return true;
}

LANE Planner::getLANE(double lane)
{
	if (lane == 1)
		return LANE::CENTER;
	else if (lane == 0)
		return LANE::LEFT;
	return LANE::RIGHT;
}
//FSM control
void Planner::DecideState(Road myRoad, double lane , double car_s)
{
	LANE currlane = getLANE(lane);
	// check if blocked, i.e. car is within 40 meters
	if (safe_lane(myRoad.get_lane_status(currlane), car_s))
	{ // if lane safe keep lane and set target high speed 
		mystate = STATE::KEEP_LANE;
		newlane = currlane;
		//target_vehicle_speed = 22.352 - 0.5;
		return;
	}
	else  //unsafe
	{
		if (safe_lane(myRoad.get_lane_status(LANE::CENTER), car_s))
		{
			if (currlane == LANE::LEFT)
			{
				mystate = STATE::CHANGE_RIGHT;
				newlane = LANE::CENTER;
				return;
			}
			else if (currlane == LANE::RIGHT)
			{
				mystate = STATE::CHANGE_LEFT;
				newlane = LANE::CENTER;
				return;
			}
			//Left or right lane and cant change to center lane
			mystate = STATE::KEEP_LANE;
			newlane = currlane;
			reducespeed = true;
			return;
		}
		// we are in the center lane and unsafe
		if (safe_lane(myRoad.get_lane_status(LANE::RIGHT), car_s))
		{
			mystate = STATE::CHANGE_RIGHT;
			newlane = LANE::RIGHT;
			return;
		}
		if (safe_lane(myRoad.get_lane_status(LANE::LEFT), car_s))
		{
			mystate = STATE::CHANGE_RIGHT;
			newlane = LANE::LEFT;
			return;
		}
		mystate = STATE::KEEP_LANE;
		newlane = currlane;
		reducespeed=true;
		return;
	}
}

/* JMT*/
vector<double> Planner::JMT(vector<double> start, vector <double> end, double T) {
	
	// prepare matrix A with coefficents
	Eigen::MatrixXd A(3, 3);
	Eigen::MatrixXd B(3, 1);

	A << T*T*T, T*T*T*T, T*T*T*T*T,
		3 * T*T, 4 * T*T*T, 5 * T*T*T*T,
		6 * T, 12 * T*T, 20 * T*T*T;

	B << end[0] - (start[0] + start[1] * T + .5*start[2] * T*T),
		end[1] - (start[1] + start[2] * T),
		end[2] - start[2];

	Eigen::MatrixXd Ai = A.inverse();
	Eigen::MatrixXd C = Ai*B;

	return { start[0], start[1], .5*start[2], C.data()[0], C.data()[1], C.data()[2] };
}


/* FSM *
void Planner::ApplyState(LANE current_lane) {
	if (current_lane == target_lane) {
		this->mystate = STATE::KEEP_LANE;
	}
	else {
		// not equal
		if (current_lane == LANE::LEFT) {
			this->mystate = STATE::CHANGE_RIGHT;
		}
		else if (current_lane == LANE::RIGHT) {
			this->mystate = STATE::CHANGE_LEFT;
		}
		else {
			if (target_lane == LANE::LEFT) {
				this->mystate = STATE::CHANGE_LEFT;
			}
			else {
				this->mystate = STATE::CHANGE_RIGHT;
			}
		}
	}
}*

/* APPLY ACTION *
void Planner::apply_action(Vehicle& car, LANE current_lane, LANE target_lane) {
	car.set_previous_s(this->end_s);
	car.set_previous_d(this->end_d);
	this->set_state(current_lane, target_lane);
}*/

/* ACTIONS */
void Planner::start_car(Vehicle& car) {
	cout << "ACTION: start_car" << endl;
	this->n = 4 * POINTS; // 4 cycles to start
	double target_v = SPEED_LIMIT*0.5;
/*	double target_s = car.get_s() + n * AT * target_v;;

	this->start_s = { car.get_s(), car.get_v(), 0.0 };
	this->end_s = { target_s, target_v, 0.0 };

	this->start_d = { get_lane_d(car.lane()), 0.0, 0.0 };
	this->end_d = { get_lane_d(car.lane()), 0.0, 0.0 };

	this->apply_action(car, car.lane(), car.lane());
	*/
}
/*
void Planner::stay_in_lane(Vehicle& car) {
	cout << "ACTION: stay_in_lane" << endl;
	this->n = CYCLES*POINTS;
	double target_v = min(car.prev_s()[1] * 1.3, SPEED_LIMIT);
	double target_s = car.prev_s()[0] + n * AT * target_v;

	this->start_s = { car.prev_s()[0], car.prev_s()[1], car.prev_s()[2] };
	this->end_s = { target_s, target_v, 0.0 };

	double target_d = get_lane_d(car.prev_d()[0]);

	this->start_d = { get_lane_d(car.prev_d()[0]), 0.0, 0.0 };
	this->end_d = { target_d, 0.0, 0.0 };

	this->apply_action(car, get_lane(car.prev_d()[0]), get_lane(car.prev_d()[0]));
}

void Planner::reduce_speed(Vehicle& car) {
	cout << "ACTION: reduce_speed" << endl;
	this->n = CYCLES*POINTS;
	this->new_points = true;
	double target_v = max(car.prev_s()[1] * 0.8, SPEED_LIMIT / 2);
	double target_s = car.prev_s()[0] + n * AT * target_v;

	this->start_s = { car.prev_s()[0], car.prev_s()[1], car.prev_s()[2] };
	this->end_s = { target_s, target_v, 0.0 };

	double target_d = get_lane_d(car.prev_d()[0]);

	this->start_d = { get_lane_d(car.prev_d()[0]), 0.0, 0.0 };
	this->end_d = { target_d, 0.0, 0.0 };

	this->apply_action(car, get_lane(car.prev_d()[0]), get_lane(car.prev_d()[0]));
}

void Planner::change_lane(Vehicle& car, LANE target_lane) {
	cout << "ACTION: reduce_speed" << endl;
	this->n = CYCLES*POINTS;
	this->new_points = true;
	double target_v = car.prev_s()[1];
	double target_s = car.prev_s()[0] + n * AT * target_v;

	this->start_s = { car.prev_s()[0], car.prev_s()[1], car.prev_s()[2] };
	this->end_s = { target_s, target_v, 0.0 };

	double target_d = get_lane_d(target_lane);

	this->start_d = { get_lane_d(car.prev_d()[0]), 0.0, 0.0 };
	this->end_d = { target_d, 0.0, 0.0 };

	this->apply_action(car, get_lane(car.prev_d()[0]), get_lane(target_d));
}*/
