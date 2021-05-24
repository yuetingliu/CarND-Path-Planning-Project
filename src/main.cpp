#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "util.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // set some parameters
  int lane = 1;
  // reference velocity
  double ref_vel = 0;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  // double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // get lane speed for later reference
          // in case of lane change, if more than one lane is safe to change
          // choose the faster lane
          map<int, float> lane_speeds;
          // find a car in the lane and get the speed
          for (int i=0; i<sensor_fusion.size(); i++) {
            float d = sensor_fusion[i][6];
            double vx, vy, vel;
            vx = sensor_fusion[i][3];
            vy = sensor_fusion[i][4];
            vel = sqrt(vx*vx + vy*vy);
            if ((d == 0) || (d == 1) || (d == 2)){
              lane_speeds[(int)d] = vel;
            }
            // if all three lane speeds are stored, no need to keep iterating
            if (lane_speeds.size() == 3) {
              break;
            }
          }

          int prev_size = previous_path_x.size();

          // use previous path end point for smooth transition
          if (prev_size > 0) {
            car_s = end_path_s;
          }

          /**
           * go through all vehicles on road from sensor fusion data
           * the logic is the following:
           * - keep lane if possible
           * - if front car is slow and distance gap is too small, try to change lane
           * - check whether lane change is possible and how many options
           * - if no lane change is possible, flag too_close and slow down
           * - if one lane change is possible, change to that lane
           * - if two lane changes are possbile, choose the faster one
           */
          // flag too_close and trigger lane change if possible
          bool too_close = false;
          // store space gap value and base braking speed on it
          float space_gap;
          for (int i=0; i<sensor_fusion.size(); i++) {
            float d = sensor_fusion[i][6];
            // check whether the car is in the same lane
            if (d < (4*(lane+1)) && d>(4*lane)) {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];

              // project s using previous path points
              check_car_s += ((double)prev_size*0.02*check_speed);
              // check whether the car is in front and is too close
              if ((check_car_s > car_s) && (check_car_s-car_s) < 30){
                space_gap = check_car_s - car_s;
                // check possible successor states for lane change
                map<string, int> successor_states = get_successor_states(lane);
                vector<int> safe_lanes;
                for (map<string, int>::iterator it=successor_states.begin();
                     it != successor_states.end(); ++it){
                  int intended_lane = it->second;
                  bool safe_to_change = true;
                  // check whether it is possible to change to the intended lane
                  for (int i=0; i<sensor_fusion.size(); i++) {
                    float d = sensor_fusion[i][6];
                    if (d < (4*(intended_lane+1)) && d>(4*intended_lane)) {
                      double vx2 = sensor_fusion[i][3];
                      double vy2 = sensor_fusion[i][4];
                      double check_speed2 = sqrt(vx2*vx2 + vy2*vy2);
                      double check_car_s2 = sensor_fusion[i][5];

                      // project s using previous path points
                      check_car_s2 += ((double)prev_size*0.02*check_speed2);
                      // check gap if no car between a distance range
                      // (-10, +30) relative to car s, then it is safe
                      if (((check_car_s2 > car_s) && (check_car_s2-car_s) < 30) ||
                          ((check_car_s2 < car_s) && (car_s - check_car_s2) < 10)){
                        safe_to_change = false;
                      }
                    }
                  }
                  if (safe_to_change) {
                    safe_lanes.push_back(intended_lane);
                  }
                }
                if (safe_lanes.size() == 0) {
                  // no safe lanes to change, keep lane and slow down
                  too_close = true;
                } else if (safe_lanes.size() == 1) {
                  lane = safe_lanes[0];
                } else if (safe_lanes.size() == 2) {
                  // both lanes can be changed, choose the faster lane
                  double lane_speed1, lane_speed2;
                  lane_speed1 = lane_speeds[safe_lanes[0]];
                  lane_speed2 = lane_speeds[safe_lanes[1]];
                  if (lane_speed1 >= lane_speed2) {
                    lane = safe_lanes[0];
                  } else {
                    lane = safe_lanes[1];
                  }
                }
              }
            }
          }

          if (too_close) {
            // lower speed based on space gap
            if (space_gap < 10) {
              // brake hard
              ref_vel -= 2.5;
            } else if (space_gap < 20) {
              // brake normaly
              ref_vel -= 1.0;
            } else {
              // brake slowly
            ref_vel -= .6;
            }
          }
          else if (ref_vel < 30.0) {
            // gas up quickly and take accel/jerk into consideration
            ref_vel += (ref_vel/25.0 + 0.5);
          }
          else if (ref_vel < 49){
            // gas up gradually
            ref_vel += 0.5;
          }
          else if (ref_vel < 49.5){
            // gas up slowly to avoid speeding ticket
            ref_vel += 0.1;
          }


          json msgJson;

          // data points for cars to drive
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // points to fit spline
          vector<double> ptsx;
          vector<double> ptsy;

          // calculate path points
          // set three points with even space and fit spline
          // interpolate points in the middle to form a list of points
          // set reference x,y yaw states
          // either reference the starting point or previous path end points
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous points are less than 2, use the car as starting references
          if (prev_size < 2) {
            // use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // use the previous path's end point as starting reference
          else {
            // redefine reference state as previous path and point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // add three points that are evenly spaced as anchor points
          vector<double> next_wp0 = getXY(
            car_s+40, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(
            car_s+80, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(
            car_s+120, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          // add the three anchor points
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i=0; i<ptsx.size(); i++) {
            // shift car reference angle to 0 degree to avoid vertical line
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
          }

          // create a spline
          tk::spline s;

          // fit spline with all five points
          s.set_points(ptsx, ptsy);

          // start with all of the previous path points from last time
          for (int i=0; i<previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // set a distance and interpolate points in between
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x*target_x) + (target_y*target_y));


          // set total to 50 points
          // here only interpolate the rest points
          for (int i=0; i<=50-previous_path_x.size(); i++) {
            double N = (target_dist)/(0.02*ref_vel/2.24);
            double x_point = (i+1)*(target_x)/N;
            double y_point = s(x_point);

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to normal after rotating it earlier
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
