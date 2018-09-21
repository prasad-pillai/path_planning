//
// Created by sree on 18/9/18.
//

#ifndef UTILS_H
#define UTILS_H


#include <cmath>
#include <math.h>
#include <vector>

using namespace std;

using std::vector;


// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }

inline double deg2rad(double x) { return x * pi() / 180; }

double findLane(double dist_from_center);

// -- helper functions from main.cpp --
double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, vector<double> &maps_x, vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);


#endif
