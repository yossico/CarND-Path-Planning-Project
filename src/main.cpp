#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "defs.h"
#include "Points.h"
#include "road.h"
#include "planner.h"

const double MAX_SPEED = 20;

using namespace std;
using namespace tk;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
const double LANEWIDTH = 4;
double ref_velocity = 0;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}


double lane = 1.0;

int main() {
	uWS::Hub h;
	bool bfirst = true;
	string map_file_ = "../data/highway_map.csv";
	Road myroad;
	Planner myPlanner;
	Points myPoints(map_file_);
	Vehicle car;


	h.onMessage([&myroad, &myPlanner, &myPoints, &car /*&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy*/](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
		uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);
		//cout << sdata << endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{

			auto s = hasData(data);

			if (s != "") {
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry") {
					// j[1] is the data JSON object

					  // Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];

					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values 
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];
					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
					double target_v = 0;
					int prev_size = previous_path_x.size();

					//debug
					cout << "car_x, car_y, car_speed, car_s, car_d, car_yaw " << car_x << " " << car_y << " " << car_speed << " " << car_s << " " << car_d << " " << car_yaw << endl;

					
					car.update(car_x, car_y, car_speed, car_s, car_d, car_yaw);
				
					vector<Vehicle> left_lane;
					vector<Vehicle> center_lane;
					vector<Vehicle> right_lane;
					vector<double> next_x_vals;
					vector<double> next_y_vals;
		
					for (int i = 0; i < sensor_fusion.size(); i++)
					{
						int id = sensor_fusion[i][0];
						double x = sensor_fusion[i][1];
						double y = sensor_fusion[i][2];
						double vx = sensor_fusion[i][3];
						double vy = sensor_fusion[i][4];
						double s = sensor_fusion[i][5];
						double d = sensor_fusion[i][6];
						double v = sqrt(vx*vx + vy*vy);

						Vehicle vehicle(id, x, y, v, s, d);
						LANE vehicle_lane = vehicle.lane();

						if (vehicle_lane == LANE::LEFT) {
							left_lane.push_back(vehicle);
						}
						else if (vehicle_lane == LANE::CENTER) {
							center_lane.push_back(vehicle);
						}
						else {
							right_lane.push_back(vehicle);
						}			
					}
					myroad.update_road(left_lane, center_lane, right_lane);

					int n = previous_path_x.size();
					for (int i = 0; i < n; i++) {
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}

					// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
					vector<vector<double>> trajectory = { next_x_vals, next_y_vals };
					myPlanner.UpdatePath(myPoints, myroad, car, trajectory);
				
					//create the control/utput msg 
					json msgJson;
					msgJson["next_x"] = trajectory[0];
					msgJson["next_y"] = trajectory[1];

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

				}
			}
			else
			{
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
	}
});

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
