#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

#include "constants.hpp"
#include "utils.hpp"

using namespace std;

// for convenience
using json = nlohmann::json;

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

// checkLaneSafety --  returns true if it is safe to go to the given lane
/// @param [in] num_points - num of points to project speed for
/// @param [in] car_s - our cars s
/// @param [in] lane_to_check - lane to check for
/// @param
bool checkLaneSafety(const int num_points, const double car_s, const double lane_to_check, const vector<vector<double> > &sensor_fusion_data) {

    double shortest_front = 100000;
    double shortest_back = -100000;

    bool good_to_change_lane = false;

    // Calculate the closest Front and Back gaps
    for (int i = 0; i < sensor_fusion_data.size(); i++) {
        float d = sensor_fusion_data[i][6];
        double carx_lane = findLane(d);
        // if other car is in the lane of interest
        if (carx_lane == lane_to_check) {

            double vx = sensor_fusion_data[i][3];
            double vy = sensor_fusion_data[i][4];
            double carx_s = sensor_fusion_data[i][5];
            double carx_speed = sqrt(vx * vx + vy * vy);

            // project futute motion
            carx_s += ((double) num_points * TIME_GAP * carx_speed);

            // find distance from our car
            double dist_s = carx_s - car_s;

            if (dist_s > 0) {                  // FRONT gap
                shortest_front = min(dist_s, shortest_front);
            } else if (dist_s <= 0) {          // BACK gap
                shortest_back = max(dist_s, shortest_back);
            }
        }
    }
    // return is safe or not
    if ((shortest_front > LANE_CHANGE_FRONT_DIST) &&
        (-1 * shortest_back > LANE_CHANGE_BACK_DIST)) {
        good_to_change_lane = true;
    }
    return good_to_change_lane;

}

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

    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](
            uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

                    json msgJson;

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

                    double reference_x;
                    double reference_y;
                    double reference_yaw;

                    int previous_path_size = previous_path_x.size();
                    int num_other_cars = sensor_fusion.size();
                    int next_waypoint = -1;
                    int lane_change_waypoint = 0;


                    // -----------------GO WITH PREVIOUS PATHS OR GO WITH NEW--------------------
                    // if there is enough previous path
                    if (previous_path_size > 2) {
                        reference_x = previous_path_x[previous_path_size - 1];
                        reference_y = previous_path_y[previous_path_size - 1];

                        double reference_x_prev = previous_path_x[previous_path_size - 2];
                        double reference_y_prev = previous_path_y[previous_path_size - 2];

                        reference_yaw = atan2(reference_y - reference_y_prev, reference_x - reference_x_prev);

                        next_waypoint = NextWaypoint(reference_x, reference_y, reference_yaw, map_waypoints_x,
                                                     map_waypoints_y);

                        // Get previous s and speed
                        car_s = end_path_s;
                        double dist = distance(reference_x_prev, reference_y_prev, reference_x, reference_y);
                        car_speed = (dist / TIME_GAP) * MS2MPH;

                    } else {
                        reference_x = car_x;
                        reference_y = car_y;
                        reference_yaw = deg2rad(car_yaw);

                        next_waypoint = NextWaypoint(reference_x, reference_y, reference_yaw, map_waypoints_x,
                                                     map_waypoints_y);

                    }

                    // -----------------DETERMINE THE RELATIVE POSITION OF THE NEARBY CARS--------------------
                    double closest_distance = 100000;
                    bool do_lane_change = false;
                    bool is_too_close = false;

                    for (int i = 0; i < num_other_cars; i++) {
                        double carx_d = sensor_fusion[i][6];    // get other car's distance
                        double carx_lane = findLane(carx_d);    // other car's lane number
                        double carx_vx = sensor_fusion[i][3];
                        double carx_vy = sensor_fusion[i][4];
                        double carx_s = sensor_fusion[i][5];

                        // if other car is in our cars lane
                        if (carx_lane == LANE) {

//                            cout << " Car in the same lane detected "  << endl;
                            double carx_speed = sqrt(carx_vx * carx_vx + carx_vy * carx_vy);
                            // predict where other car will be in the next iteration
                            carx_s += (previous_path_size * TIME_GAP * carx_speed);
                            double front_gap = carx_s - car_s;

                            // check other cars distance from ours
                            if (front_gap > 0 && front_gap < FRONT_SAFE_DISTANCE && front_gap < closest_distance) {
                                closest_distance = front_gap;

                                // if there is not enough space in the front
                                if (front_gap > FRONT_TOO_CLOSE_DIST) {
//                                    TOP_SPEED = carx_speed * MS2MPH; // go at max possible speed
                                    TOP_SPEED = MAX_POSSIBLE_SPEED * MS2MPH; // go at max possible speed
                                    do_lane_change = true;      // lane can be changed
                                    cout << " Other car is much further, going at maximum speed" << endl;
                                } else {  // not enough space in the front
                                    // go slower than front car
                                    TOP_SPEED = carx_speed * MS2MPH - 5.0;
                                    is_too_close = true;
                                    do_lane_change = true;      //lane can be changed
                                    cout << " Other car is very close, going slower"
                                    << ", current speed: "
                                    << car_speed << endl;
                                }
                            }
                        }
                    }

                    //------------------------------LANE SHIFTING LOGIC------------------------------
                    int delta_waypoint = next_waypoint - lane_change_waypoint;
                    int remaining_waypoint = delta_waypoint % map_waypoints_x.size();

                    if (do_lane_change && remaining_waypoint > 2) {

//                        cout << " Lane change possibility detected " << endl;
                        bool did_change_lane = false;

                        // Check if car not in the left most lane
                        if (LANE != LEFT_LANE && !did_change_lane) {
                            bool lane_safe = true;

                            // check if it is good to change to left lane
                            lane_safe = checkLaneSafety(previous_path_size,
                                                        car_s,
                                                        LANE - 1, // change to the left of the current lane
                                                        sensor_fusion);

                            if (lane_safe) { //change to the left lane
                                // set other flags
                                did_change_lane = true;
                                LANE -= 1;
                                lane_change_waypoint = next_waypoint;
                            }
                        }
                        // Check if car not in the right most lane
                        if (LANE != RIGHT_LANE && !did_change_lane) {

                            // check if it is good to change to right lane
                            bool lane_safe = checkLaneSafety(previous_path_size,
                                                             car_s,
                                                             LANE + 1,  // change to the right of the current lane
                                                             sensor_fusion);

                            if (lane_safe) { //change to the left lane
                                // set other flags
                                did_change_lane = true;
                                LANE += 1;
                                lane_change_waypoint = next_waypoint;
                            }
                        }
                        if (did_change_lane) {
                            cout << " Lane changed " << endl;
                        }

                    }

                    //------------------------------CREATE CONTROL POINTS------------------------------
                    vector<double> control_point_x;
                    vector<double> control_point_y;

                    // if previous path size does not have enough values
                    if (previous_path_size < 2) {

                        // use two points that make a tangent path to the car
                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);

                        // add them to anchor points
                        control_point_x.push_back(prev_car_x);
                        control_point_x.push_back(car_x);

                        control_point_y.push_back(prev_car_y);
                        control_point_y.push_back(car_y);

                    } else {

                        // add them to anchor points
                        control_point_x.push_back(previous_path_x[previous_path_size - 2]);
                        control_point_x.push_back(previous_path_x[previous_path_size - 1]);

                        control_point_y.push_back(previous_path_y[previous_path_size - 2]);
                        control_point_y.push_back(previous_path_y[previous_path_size - 1]);
                    }

                    double target_lane_dist = 2 + LANE * 4;  // d coordinate for target lane

                    // finding next waypoints
                    vector<double> waypoint0 = getXY((car_s + NUM_WAYPOINTS), target_lane_dist, map_waypoints_s,
                                                     map_waypoints_x,
                                                     map_waypoints_y);
                    vector<double> waypoint1 = getXY((car_s + NUM_WAYPOINTS * 2), target_lane_dist, map_waypoints_s,
                                                     map_waypoints_x,
                                                     map_waypoints_y);
                    vector<double> waypoint2 = getXY((car_s + NUM_WAYPOINTS * 3), target_lane_dist, map_waypoints_s,
                                                     map_waypoints_x,
                                                     map_waypoints_y);

                    // add them to the control points
                    control_point_x.push_back(waypoint0[0]);
                    control_point_y.push_back(waypoint0[1]);

                    control_point_x.push_back(waypoint1[0]);
                    control_point_y.push_back(waypoint1[1]);

                    control_point_x.push_back(waypoint2[0]);
                    control_point_y.push_back(waypoint2[1]);

                    // transform from fernet to local coordinates
                    for (int i = 0; i < control_point_x.size(); i++) {
                        double move_x = control_point_x[i] - reference_x;
                        double move_y = control_point_y[i] - reference_y;

                        // rotations
                        control_point_x[i] = (move_x * cos(0 - reference_yaw) - move_y * sin(0 - reference_yaw));
                        control_point_y[i] = (move_x * sin(0 - reference_yaw) + move_y * cos(0 - reference_yaw));
                    }

                    // creating the spline using the helper library
                    tk::spline path_spline;

                    // add control points on the spline
                    path_spline.set_points(control_point_x, control_point_y);

                    // add control from previous paths also
                    for (int i = 0; i < previous_path_size; i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    // get x, y
                    double target_x = NUM_WAYPOINTS;
                    double target_y = path_spline(target_x);
                    double target_distance = sqrt((target_x * target_x) + (target_y * target_y));

                    double increment = 0;

                    // filling rest of the path
                    for (int i = 1; i < 50 - previous_path_size; i++) {
                        // if car is too slow, accelerate
                        if (car_speed < TOP_SPEED) {
                            car_speed += (MS2MPH / 10);     // 0.224;
                        } // else decelerate
                        else if (car_speed > TOP_SPEED) {
                            car_speed -= (MS2MPH / 10);    // 0.224;
                        }
                        // calculate the waypoint spacing based on the target velocity
                        double N = (target_distance / (TIME_GAP * car_speed / MS2MPH)); // num of points
                        double point_x = increment + (target_x) / N;
                        double point_y = path_spline(point_x);

                        increment = point_x;

                        double x_ref = point_x;
                        double y_ref = point_y;

                        // transform coordinates back to normal
                        point_x = (x_ref * cos(reference_yaw) - y_ref * sin(reference_yaw));
                        point_y = (x_ref * sin(reference_yaw) + y_ref * cos(reference_yaw));

                        point_x += reference_x;
                        point_y += reference_y;

                        next_x_vals.push_back(point_x);
                        next_y_vals.push_back(point_y);

                    }

                    // done my code

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

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
    h.onHttpRequest([](
            uWS::HttpResponse *res, uWS::HttpRequest
    req,
            char *data,
            size_t, size_t
    ) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
// i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](
            uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest
    req) {
        std::cout << "Connected!!!" <<
                  std::endl;
    });

    h.onDisconnection([&h](
            uWS::WebSocket<uWS::SERVER> ws,
            int code,
            char *message, size_t
            length) {
        ws.close();

        std::cout << "Disconnected" <<
                  std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port <<
                  std::endl;
    } else {
        std::cerr << "Failed to listen to port" <<
                  std::endl;
        return -1;
    }
    h.run();
}
