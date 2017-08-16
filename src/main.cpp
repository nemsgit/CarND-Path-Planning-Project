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
#include "helper.h"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  // define target lane number (the leftmost is lane 0) and target velocity
  int target_lane = 1;
  double target_v = 0.0; // in m/s


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,
              &target_lane, &target_v](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
            /**
             * Read in Data
             */
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
          	auto sensor_fusion = j[1]["sensor_fusion"];

            /**
             * Determine Target Lane and Target Speed
             */
            int prev_size = previous_path_x.size();

            // flags related to lane changing
            bool too_close = false;
            bool left_lane_clear = true;
            bool right_lane_clear = true;

            // reference s position of the ego car (at the end of previous path)
            double ego_s = (prev_size > 0)? end_path_s:car_s;
            // record speed of the front car when it's too close
            double front_car_v;

            // loop through sensor fusion list and configure the lane-changing flags
            // according to the traffic
            for (int i = 0; i < sensor_fusion.size(); ++i) {
                // read velocity
                double vx1 = sensor_fusion[i][3];
                double vy1 = sensor_fusion[i][4];
                double v1 = sqrt(vx1 * vx1 + vy1 * vy1);
                // read current s value and calculate future s position of the car to compare with
                // the ego car
                double s1 = sensor_fusion[i][5];
                s1 += (double)prev_size * TIME_STEP * v1;
                // read d value and calculate lane number
                double d1 = sensor_fusion[i][6];
                //int lane1 = (int)(d1 / 4.0);

                // if the car is in the same lane and too close ahead of the ego car, set too_close.
                // note we make the ego lane wider (6m) in this case to account for
                // cars in the middle of lane changing.
                if ((d1 < (target_lane + 1) * 4 + 1) && (d1 > target_lane * 4 - 1)
                    && (s1 > ego_s) && (s1 - ego_s < 30)){
                    front_car_v = v1;
                    too_close = true;
                }
                // if the car is not in the same lane but very close to the ego car in s distance
                else if ((s1 - ego_s < 32) && (s1 - ego_s > -7)) {
                    if ((d1 <= target_lane * 4) && (d1 >= (target_lane - 1) * 4)) {
                        left_lane_clear = false;
                    }
                    else if ((d1 >= (target_lane + 1) * 4) && (d1 <= (target_lane + 2) * 4)) {
                        right_lane_clear = false;
                    }
                }
            }

            // update target lane and velocity according to the lane changing flags
            if (too_close) {
                if (left_lane_clear && (target_lane > 0)){
                    target_lane -= 1;
                }
                else if (right_lane_clear && (target_lane < 2)){
                    target_lane += 1;
                }
                // if neighboring lanes are busy, slow down.
                else if (target_v > front_car_v){
                    target_v -= 0.07;
                }
            }

            else if (target_v < 0.99 * SPEED_LIMIT * MILE2METER) {
                target_v += 0.1;
            }

            /**
             * Spline Fitting
             * we'll be using five points to fit the spline. The first two are the last
             * two points from previous_path (if it contains more than 2 points) or the
             * ego car's current and previous positions (if the previous_path has less than
             * 2 points left). The next 2 points are some distance away down the road
             * in the middle of the target_lane.
             */
            vector<double> ptsx, ptsy;

            // prepare the first 2 points and append to the vector
            double x1, x2, y1, y2;
            if (prev_size >= 2) {
                x2 = previous_path_x[prev_size - 1];
                y2 = previous_path_y[prev_size - 1];
                x1 = previous_path_x[prev_size - 2];
                y1 = previous_path_y[prev_size - 2];
            }
            else {
                x2 = car_x;
                y2 = car_y;
                x1 = x2 - cos(deg2rad(car_yaw)); // ego car's "previous" x position assuming v*delta_t = 1
                y1 = y2 - sin(deg2rad(car_yaw));
            }

            ptsx.push_back(x1);
            ptsx.push_back(x2);
            ptsy.push_back(y1);
            ptsy.push_back(y2);

            // prepare the next 2 points and append to the vector
            double target_d = 4 * target_lane + 2;
            for (int i = 0; i < 2; ++i) {
                double target_s = car_s + (i + 1) * 50;
                // transform (target_s, target_d) from s-d to x-y coordinates
                vector<double> xy = getXY(target_s, target_d, map_waypoints_s,
                                          map_waypoints_x, map_waypoints_y);
                ptsx.push_back(xy[0]);
                ptsy.push_back(xy[1]);
            }

            // in addition to the 5 points, we also need the tangential angle at (x2, y2) to
            // convert all global coordinates to local coordinates
            double tan_angle = atan2(y2 - y1, x2 - x1);

            // transform all points to local coordinates using (x2, y2) as origin
            // and tan_angle as reference angle
            for (int i = 0; i < ptsx.size(); ++i) {
                double dx = ptsx[i] - x2;
                double dy = ptsy[i] - y2;
                ptsx[i] = dx * cos(-tan_angle) - dy * sin(-tan_angle);
                ptsy[i] = dx * sin(-tan_angle) + dy * cos(-tan_angle);
            }

            // define and fit a spline
            tk::spline spl;
            spl.set_points(ptsx, ptsy);

            /**
             * Spline Partition
             * use the method Aaron Brown mentioned in the lecture to partition the spline
             * we look at some horizon length (hori_x) away down the road, where the ego-car
             * reaches a new position at (hori_x, hori_y) in the local frame.
             */
            double hori_x = 30.0;
            double hori_y = spl(hori_x);

            // the car has just traveled a distance of (use Euclidean distance to
            // approximate the arc-length)
            double hori_dist = distance(hori_x, hori_y, 0, 0);

            // when the ego car travels at target_v, this length should be partitioned
            // into n segments, where
            double n = (hori_dist / (target_v * TIME_STEP));
            // correspondingly, we will divide hori_x into N segments and append the points
            // on the spline to the new path

            /**
             * Create the New Path
             */
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            // first feed it with the residual points from the previous path
            for (int i = 0; i < prev_size; ++i) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            // append new points from the spline
            for (int i = 0; i < PATH_LENGTH - prev_size; ++i){
                // calculate (x, y) in local coordinates
                double x_local = (i + 1) * (hori_x / n);
                double y_local = spl(x_local);
                // transform to global coordinates (note (x2, y2) is the global coordinates
                // of the local origin)
                double x_global = x_local * cos(tan_angle) - y_local * sin(tan_angle);
                double y_global = x_local * sin(tan_angle) + y_local * cos(tan_angle);
                x_global += x2;
                y_global += y2;
                // append the global coordinates to new path
                next_x_vals.push_back(x_global);
                next_y_vals.push_back(y_global);
            }

            /**
             * Send Data back to Simulator
             */

            json msgJson;
            msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
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


