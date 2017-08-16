//
// Created by daihua on 17-8-13.
//

#ifndef PATH_PLANNING_HELPER_H
#define PATH_PLANNING_HELPER_H

#include <fstream>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;

// time interval of adjacent path points
const double TIME_STEP = 0.02;

// number of points in a new path
const int PATH_LENGTH = 50;

// converting from mph to m/s
const double MILE2METER = 0.44704;

// speed limit in mph
const double SPEED_LIMIT = 50.0;

// For converting back and forth between radians and degrees.
double deg2rad(double x);
double rad2deg(double x);

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s);

// Calculate Euclidean distance
double distance(double x1, double y1, double x2, double y2);

// Find index of the closest way point
int ClosestWaypoint(double x, double y, vector<double> maps_x,
                    vector<double> maps_y);

// Find the next way point
int NextWaypoint(double x, double y, double theta, vector<double> maps_x,
                 vector<double> maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x,
                         vector<double> maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s,
                     vector<double> maps_x, vector<double> maps_y);


#endif //PATH_PLANNING_HELPER_H
