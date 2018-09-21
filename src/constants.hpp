//
// Created by sree on 18/9/18.
//

#ifndef PATH_PLANNING_CONSTANTS_H
#define PATH_PLANNING_CONSTANTS_H

const double MAX_VEL = 48.0;
double REF_VEL = 0.0;

const double TIME_GAP = 0.02;       // time gap between each fetch
const double MS2MPH = 2.24;         // conversion factor for meter per sec to miles per hour

int LANE = 1;

const double FRONT_SAFE_DISTANCE = 35.0;            // meters;

const double LANE_CHANGE_FRONT_DIST = 35.0; // safe distance from a car at front in meters
const double LANE_CHANGE_BACK_DIST = 13.0; // safe distance from a car at back in meters

const double NUM_WAYPOINTS = 30;    // Spacing (m) in waypoints

#endif //PATH_PLANNING_CONSTANTS_H
