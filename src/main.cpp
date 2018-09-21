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
bool checkLaneSafety(const int num_points, const double car_s, const double lane_to_check,
                     const vector<vector<double> > &sensor_fusion_data) {

    double shortest_front = 100000;
    double shortest_back = -100000;

    bool good_to_change_lane = false;

    // Calculate the closest Front and Back gaps
    for (int i = 0; i < sensor_fusion_data.size(); i++) {
        float d = sensor_fusion_data[i][6];
        double carx_lane = findLane(d);
        // if other car is in the lane of interest
        if (carx_lane == lane_to_check) {

            double carx_vx = sensor_fusion_data[i][3];
            double carx_vy = sensor_fusion_data[i][4];
            double carx_s = sensor_fusion_data[i][5];
            double carx_speed = sqrt(carx_vx * carx_vx + carx_vy * carx_vy);

            // project future motion
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

                    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

                    // My code

                    int prev_size = previous_path_x.size();
                    bool is_too_close = false;
                    bool do_lane_change = false;
                    bool is_lane_changed = false;
                    int next_waypoint = -1;

                    if (prev_size > 0) {
                        car_s = end_path_s;
                    }

                    for (int i = 0; i < sensor_fusion.size(); i++) {
                        // cout << "inside SF loop" << endl;
                        float check_car_d = sensor_fusion[i][6];
                        double check_car_lane = findLane(check_car_d);
                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double check_car_s = sensor_fusion[i][5];

                        // other car is in my lane
                        if (check_car_lane == LANE) {

                            double check_car_speed = sqrt(vx * vx + vy * vy);
                            check_car_s += ((double) prev_size * TIME_GAP * check_car_speed);

                            double front_gap = check_car_s - car_s;

                            if ((check_car_s > car_s) && (front_gap < FRONT_SAFE_DISTANCE)) {
                                is_too_close = true;
                                do_lane_change = true;
                                if (REF_VEL > check_car_speed * MS2MPH - 2.0) {
                                    REF_VEL -= .6;
                                }
                            } else { // enough space in front go full speed
                                if (REF_VEL < MAX_VEL) {
                                    REF_VEL += .224 * .5;
                                }

                            }

                        }
                    }

                    bool is_lane_safe;
                    if (do_lane_change) {
                        // cout << "try lane shift" << endl;
                        // cout << "current lane: " << LANE << endl;
                        if (LANE == 0) { // in left most lane
                            // check if lane 1 is safe
                            is_lane_safe = checkLaneSafety(previous_path_x.size(), car_s, 1, sensor_fusion);
                            if (is_lane_safe && !is_lane_changed) {
                                LANE = 1;
                                is_lane_changed = true;
                            }
                        } else if (LANE == 2) { // in right most lane
                            // check if lane 1 is safe
                            is_lane_safe = checkLaneSafety(previous_path_x.size(), car_s, 1, sensor_fusion);
                            if (is_lane_safe && !is_lane_changed) {
                                LANE = 1;
                                is_lane_changed = true;
                            }
                        } else if (LANE == 1) {
                            //check if lane 0 is safe
                            is_lane_safe = checkLaneSafety(previous_path_x.size(), car_s, 0, sensor_fusion);
                            if (is_lane_safe && !is_lane_changed) {
                                // change to left lane
                                LANE = 0;
                                is_lane_changed = true;
                            } else {
                                // else check if lane 2 is safe
                                is_lane_safe = checkLaneSafety(previous_path_x.size(), car_s, 2, sensor_fusion);
                                if (is_lane_safe && !is_lane_changed) {
                                    // change to left lane
                                    LANE = 2;
                                    is_lane_changed = true;
                                }
                            }

                        }
                    }

                    vector<double> ptsx;
                    vector<double> ptsy;

                    double ref_x = car_x;
                    double ref_y = car_y;
                    double ref_yaw = deg2rad(car_yaw);

                    if (prev_size < 2) {
                        //Use 2 points that make the path tangent to the car
                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);

                        next_waypoint = NextWaypoint(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);

                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(car_x);

                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(car_y);
                    } else {
                        //use the previous path end points as the starting reference
                        ref_x = previous_path_x[prev_size - 1];
                        ref_y = previous_path_y[prev_size - 1];

                        double ref_x_prev = previous_path_x[prev_size - 2];
                        double ref_y_prev = previous_path_y[prev_size - 2];
                        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
                        next_waypoint = NextWaypoint(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);

                        // Use 2 points that make the path tangent to the car

                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);

                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);
                    }

                    double target_lane_dist = 2 + LANE * 4;  // d coordinate for target lane

                    // finding next waypoints
                    vector<double> next_wp0 = getXY((car_s + NUM_WAYPOINTS), target_lane_dist, map_waypoints_s,
                                                    map_waypoints_x,
                                                    map_waypoints_y);
                    vector<double> next_wp1 = getXY((car_s + NUM_WAYPOINTS * 2), target_lane_dist, map_waypoints_s,
                                                    map_waypoints_x,
                                                    map_waypoints_y);
                    vector<double> next_wp2 = getXY((car_s + NUM_WAYPOINTS * 3), target_lane_dist, map_waypoints_s,
                                                    map_waypoints_x,
                                                    map_waypoints_y);

                    ptsx.push_back(next_wp0[0]);
                    ptsx.push_back(next_wp1[0]);
                    ptsx.push_back(next_wp2[0]);

                    ptsy.push_back(next_wp0[1]);
                    ptsy.push_back(next_wp1[1]);
                    ptsy.push_back(next_wp2[1]);

                    for (int i = 0; i < ptsx.size(); i++) {
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;

                        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
                        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
                    }

                    tk::spline s;
                    s.set_points(ptsx, ptsy);

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    for (int i = 0; i < previous_path_x.size(); i++) {

                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    double target_x = 30.0;
                    double target_y = s(target_x);
                    double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

                    double x_add_on = 0;

                    for (int i = 0; i <= (50 - previous_path_x.size()); i++) {
                        double N = (target_dist / (.02 * REF_VEL / 2.24));
                        double x_point = x_add_on + (target_x) / N;
                        double y_point = s(x_point);

                        x_add_on = x_point;

                        double x_ref = x_point;
                        double y_ref = y_point;

                        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
                        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

                        x_point += ref_x;
                        y_point += ref_y;

                        // cout << "next_x_vals : " << x_point;
                        // cout << " next_y_vals : " << y_point;
                        // cout << endl;

                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);
                    }



                    // My new code ends

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
