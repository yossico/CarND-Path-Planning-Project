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
void Planner::DecideState(Road myRoad, double lane , Vehicle& car)
{
	LANE currlane = getLANE(lane);
	// check if blocked, i.e. car is within 40 meters
	if (myRoad.safe_lane(car, car.lane()))
	{ // if lane safe keep lane and set target high speed 
		mystate = STATE::KEEP_LANE;
		newlane = currlane;
		//target_vehicle_speed = 22.352 - 0.5;
		return;
	}
	else  //unsafe
	{
		if (myRoad.safe_lane(car, LANE::CENTER))
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
		if (myRoad.safe_lane(car, LANE::RIGHT))
		{
			mystate = STATE::CHANGE_RIGHT;
			newlane = LANE::RIGHT;
			return;
		}
		if ((myRoad.safe_lane(car, LANE::LEFT))
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
