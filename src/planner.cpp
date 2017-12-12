#include "defs.h"
#include "Points.h"
#include "planner.h"

using namespace std;

Planner::Planner() {
	this->state = STATE::START;
}

LANE get_lane(double d) {
	LANE lane;
	if (d < 4.0) {
		lane = LANE::LEFT;
	}
	else if ((d >= 4.0) && (d < 8.0)) {
		lane = LANE::CENTER;
	}
	else {
		lane = LANE::RIGHT;
	}
	return lane;
}

double get_lane_d(LANE lane) {
	double d;
	if (lane == LANE::LEFT) {
		d = 2.0;
	}
	else if (lane == LANE::CENTER) {
		d = 6.0;
	}
	else {
		d = 10.0;
	}
	return d;
}

double get_lane_d(double D) {
	double d;
	if (D < 4.0) {
		d = 2.0;
	}
	else if ((D >= 4.0) && (D < 8.0)) {
		d = 6.0;
	}
	else {
		d = 10.0;
	}
	return d;
}


bool Planner::safe_lane(vector<Vehicle> carsinlane, double car_s)
{
	for (Vehicle other_car : carsinlane) 
	{
		double s_diff = fabs(other_car.s - car_s);
		if (s_diff < SAFETY_DISTANCE)
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


/* JMT*/
vector<double> Planner::JMT(vector<double> start, vector <double> end, double T)
{
	cout << "JMT::start S V A" << start[0] << " " << start[1] << " " << start[2] << endl;
	cout << "JMT::end S V A" << end[0] << " " << end[1] << " " << end[2] << endl;
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
	cout << "finished JMT" << endl;
	return { start[0], start[1], .5*start[2], C.data()[0], C.data()[1], C.data()[2] };
}

void Planner::set_state(LANE current_lane, LANE target_lane) {
	if (current_lane == target_lane) {
		this->state = STATE::KEEP_LANE;
	}
	else {
		// not equal
		if (current_lane == LANE::LEFT) {
			this->state = STATE::CHANGE_RIGHT;
		}
		else if (current_lane == LANE::RIGHT) {
			this->state = STATE::CHANGE_LEFT;
		}
		else {
			if (target_lane == LANE::LEFT) {
				this->state = STATE::CHANGE_LEFT;
			}
			else {
				this->state = STATE::CHANGE_RIGHT;
			}
		}
	}
}

void Planner::UpdatePath(Points& points, Road& myRoad, Vehicle& car,  vector<vector<double>>& trajectory) 
{
	//reducespeed = false;
	LANE currlane = car.lane();
	double lanenum = car.lanenum();	
	if (trajectory[0].size() > POINTS)
	{
		return;
	}
	cout << "traj size  " << trajectory[0].size() << endl;
	cout << "car_speed, car_s, car_d " << " " << car.get_v() << " " << car.get_s() << " " << car.get_d() <<  endl;
	if (this->state == STATE::START)
	{
		this->start_car(car);
	}
	// check if blocked, car is within 40 meters
	else
	{
		if (myRoad.free_lane(car, car.lane()))
		{ // if lane safe keep lane and set target high speed 
			stay_in_lane(car);			
		}
		else  //unsafe
		{
			if (myRoad.free_lane(car, LANE::CENTER))
			{	cout << "current lane unsafe switch to center lane";
				if (currlane == LANE::RIGHT)
				{
					state = STATE::CHANGE_LEFT;
					cout << "change left" << endl;
					change_lane(car, LANE::RIGHT);
				}
				else if (currlane == LANE::LEFT)
				{
					state = STATE::CHANGE_RIGHT;
					cout << "Change Right" << endl;
					change_lane(car, LANE::RIGHT);
				}
			}
			else if ((currlane == LANE::RIGHT) || (currlane == LANE::LEFT))  //its unsafe and center lane is not free we are in center lane
			{
				//Left or right lane and cant change to center lane
				cout << "current lane && center lane unsafe stay in R/L lanes" << endl;
				state = STATE::KEEP_LANE;
				this->reduce_speed(car);
			}
			// we are in the center lane and unsafe
			else if (myRoad.free_lane(car, LANE::RIGHT))
			{
				cout << "CL RIGHT" << endl;
				state = STATE::CHANGE_RIGHT;
				change_lane(car, LANE::RIGHT);
				//state = STATE::CHANGE_RIGHT; //newlane = LANE::RIGHT;
			}
			else if (myRoad.free_lane(car, LANE::LEFT))
			{
				cout << "CL LEFT" << endl;
				state = STATE::CHANGE_RIGHT;
				change_lane(car, LANE::LEFT);
				return;
			}
			cout << "lane unsafe couldnt change lanes reduce speed" << endl;
			state = STATE::KEEP_LANE;
			this->reduce_speed(car);
		}
	}
	GetJMTPathPoints(points, trajectory);
}

void Planner::GetJMTPathPoints(Points& points, vector<vector<double>>& trajectory)
{
	// jmt
	double T = this->n * AT;
	cout << "JMT::S ";
	vector<double> poly_s = this->JMT(this->start_s, this->end_s, T);
	cout << "JMT::D ";
	vector<double> poly_d = this->JMT(this->start_d, this->end_d, T);
	cout << "n="<<n<<endl;
	double t, next_s, next_d, mod_s, mod_d;
	vector <double> XY;
	for (int i = 0; i < n; i++) 
	{
		t = AT*i;
		// /* JMT */
		// cout << "----------JMT----------" << endl;
		// cout << "t= " << t << endl;
		next_s = 0.0;
		next_d = 0.0;

		for (int a = 0; a < poly_s.size(); a++) {

			next_s += poly_s[a] * pow(t, a);
			next_d += poly_d[a] * pow(t, a);
		}
		mod_s = fmod(next_s, TRACK_DISTANCE);
		mod_d = fmod(next_d, ROAD_WIDTH);
		//cout << "t= " << t << " mod_s=" << mod_s << " mod_d= "<< mod_d<< endl;
		XY = points.getXY(mod_s, mod_d);
	
		trajectory[0].push_back(XY[0]);
		trajectory[1].push_back(XY[1]);
	}
	cout << "finished GetJMTPathPoints"<<endl;
}

/* APPLY ACTION */
void Planner::apply_action(Vehicle& car, LANE current_lane, LANE target_lane) {
	car.set_previous_s(this->end_s);
	car.set_previous_d(this->end_d);
	set_state(current_lane, target_lane);
}

/* ACTIONS */
void Planner::start_car(Vehicle& car) {
	cout << "ACTION: start_car" << endl;
	this->n = 4 * POINTS; // 4 cycles to start
	double target_v = SPEED_LIMIT*0.5;
	double target_s = car.get_s() + n * AT * target_v;;

	this->start_s = { car.get_s(), car.get_v(), 0.0 };
	this->end_s = { target_s, target_v, 0.0 };

	this->start_d = { get_lane_d(car.lane()), 0.0, 0.0 };
	this->end_d = { get_lane_d(car.lane()), 0.0, 0.0 };

	this->apply_action(car, car.lane(), car.lane());
}

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
	double target_v = max(car.get_v()*0.8, SPEED_LIMIT / 2);
	double target_s = car.get_s() + n * AT * target_v;

	this->start_s = { car.get_s(), car.get_v(), car.prev_s()[2] };
	this->end_s = { target_s, target_v, 0.0 };

	double target_d = get_lane_d(car.get_d());

	this->start_d = { car.get_d(), 0.0, 0.0 };
	this->end_d = { target_d, 0.0, 0.0 };

	this->apply_action(car, get_lane(car.get_d()), get_lane(target_d));
}

void Planner::change_lane(Vehicle& car, LANE target_lane) {
	cout << "ACTION: reduce_speed" << endl;
	this->n = CYCLES*POINTS;
	this->new_points = true;
	double target_v = car.get_v();
	double target_s = car.get_s() + n * AT * target_v;

	this->start_s = { car.get_s(), car.get_v(), car.prev_s()[2] };
	this->end_s = { target_s, target_v, 0.0 };

	double target_d = get_lane_d(target_lane);

	this->start_d = { car.get_d(), 0.0, 0.0 };
	this->end_d = { target_d, 0.0, 0.0 };

	this->apply_action(car, get_lane(car.get_s()), get_lane(target_d));
}


/* APPLY ACTION *
void Planner::apply_action(Vehicle& car, LANE current_lane, LANE target_lane) {
	car.set_previous_s(this->end_s);
	car.set_previous_d(this->end_d);
	set_state(current_lane, target_lane);
}


/* ACTIONS *
void Planner::start_car(Vehicle& car) {
	cout << "start_car" << endl;
	this->state = STATE::KEEP_LANE;
	this->start_car(car);	
	this->n = 4 * POINTS; // 4 cycles to start
	double target_v = SPEED_LIMIT*0.5;
	double target_s = car.get_s() + n * AT * target_v;;

	this->start_s = { car.get_s(), car.get_v(), 0.0 };
	this->end_s = { target_s, target_v, 0.0 };

	this->start_d = { get_lane_d(car.lane()), 0.0, 0.0 };
	this->end_d = { get_lane_d(car.lane()), 0.0, 0.0 };

	this->apply_action(car, car.lane(), car.lane());
}

void Planner::stay_in_lane(Vehicle& car) {
	cout << "ACTION: stay_in_lane" << endl;
	state = STATE::KEEP_LANE;	
	this->n = CYCLES*POINTS;
	double target_v = min(car.prev_s()[1] * 1.3, SPEED_LIMIT);
	double target_s = car.prev_s()[0] + n * AT * target_v;

	this->start_s = { car.prev_s()[0], car.prev_s()[1], car.prev_s()[2] };
	this->end_s = { target_s, target_v, 0.0 };

	double target_d = get_lane_d(car.get_d());

	this->start_d = { get_lane_d(car.get_d()), 0.0, 0.0 };
	this->end_d = { target_d, 0.0, 0.0 };

	this->apply_action(car, get_lane(car.get_d()), get_lane(car.get_d()));
}

void Planner::reduce_speed(Vehicle& car) {
	cout << "ACTION: reduce_speed" << endl;
	this->n = CYCLES*POINTS;
	this->new_points = true;
	double target_v = max(car.get_v()*0.8, SPEED_LIMIT / 2);
	double target_s = car.get_s() + n * AT * target_v;

	this->start_s = { car.get_s(), car.get_v(), car.prev_s()[2] };
	this->end_s = { target_s, target_v, 0.0 };

	double target_d = get_lane_d(car.get_d());

	this->start_d = { car.get_d(), 0.0, 0.0 };
	this->end_d = { target_d, 0.0, 0.0 };

	this->apply_action(car, get_lane(car.get_d()), get_lane(target_d));
}

void Planner::change_lane(Vehicle& car, LANE target_lane) {
	cout << "ACTION: reduce_speed" << endl;
	this->n = CYCLES*POINTS;
	this->new_points = true;
	double target_v = car.get_v();
	double target_s = car.get_s() + n * AT * target_v;

	this->start_s = { car.get_s(), car.get_v(), car.prev_s()[2] };
	this->end_s = { target_s, target_v, 0.0 };

	double target_d = get_lane_d(target_lane);

	this->start_d = { car.get_d(), 0.0, 0.0 };
	this->end_d = { target_d, 0.0, 0.0 };

	this->apply_action(car, get_lane(car.get_s()), get_lane(target_d));
}*/

