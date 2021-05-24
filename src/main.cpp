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
  double ref_vel = 0;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

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

          // get lane speed
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
          }


          int prev_size = previous_path_x.size();

          // check other cars on the road
          if (prev_size > 0) {
            car_s = end_path_s;
          }

          bool too_close = false;
          // find ref_v to use
          for (int i=0; i<sensor_fusion.size(); i++) {
            float d = sensor_fusion[i][6];
            if (d < (2+4*lane+2) && d>(2+4*lane-2)) {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];

              // project s using previous path points
              check_car_s += ((double)prev_size*0.02*check_speed);
              // check s values greater than mine and s gap
              if ((check_car_s > car_s) && (check_car_s-car_s) < 30){
                // do some logic here, lower reference speed so we don't crash
                //ref_vel = 29.5;
                // check possible successor states
                map<string, int> successor_states = get_successor_states(lane);
                vector<int> safe_lanes;
                for (map<string, int>::iterator it=successor_states.begin();
                     it != successor_states.end(); ++it){
                  int intended_lane = it->second;
                  bool safe_to_change = true;
                  // check whether it is possible to change to the intended lane
                  // no car between a distance range (-10, +30) relative to car s
                  for (int i=0; i<sensor_fusion.size(); i++) {
                    float d = sensor_fusion[i][6];
                    if (d < (2+4*intended_lane+2) && d>(2+4*intended_lane-2)) {
                      double vx = sensor_fusion[i][3];
                      double vy = sensor_fusion[i][4];
                      double check_speed = sqrt(vx*vx + vy*vy);
                      double check_car_s = sensor_fusion[i][5];

                      // project s using previous path points
                      check_car_s += ((double)prev_size*0.02*check_speed);
                      // check s values greater than mine and s gap
                      if (((check_car_s > car_s) && (check_car_s-car_s) < 30) ||
                          ((check_car_s < car_s) && (car_s - check_car_s) < 10)){
                        safe_to_change = false;
                      }
                    }
                  }
                  if (safe_to_change) {
                    safe_lanes.push_back(intended_lane);
                  }
                }
                if (safe_lanes.size() == 0) {
                  // no safe lanes to change, keep lane
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
            ref_vel -= .3;
          }
          else if (ref_vel < 49.50){
            ref_vel += .3;
          }


          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          /**
           * create a list of widely spaced (x, y) waypoints, evenly spaced at 30m
           * later we interoplate these waypoints with a spline and fill it with
           * more points
           */

          vector<double> ptsx;
          vector<double> ptsy;

          // reference x,y yaw states
          // either reference the starting point or previous path end points
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if the previous size is almost empty, use the car as starting references
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

            // use two points that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // in Frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i=0; i<ptsx.size(); i++) {
            // shift car reference angle to 0 degree
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
          }

          // create a spline
          tk::spline s;

          // set (x,y) points to the spline
          s.set_points(ptsx, ptsy);

          // start with all of the previous path points from last time
          for (int i=0; i<previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x*target_x) + (target_y*target_y));

          double x_add_on = 0;

          // fill up the rest of pur path planner after filling it with previous pooints, here will will always ooutput 50 points
          for (int i=0; i<=50-previous_path_x.size(); i++) {
            double N = (target_dist)/(0.02*ref_vel/2.24);
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

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

          /**
          double dist_inc = 0.3;
          for (int i = 0; i < 50; i++) {
            double next_s = car_s + (i+1) * dist_inc;
            double next_d = 6;
            vector<double> xy = getXY(
              next_s, next_d, map_waypoints_s,
              map_waypoints_x, map_waypoints_y
            );
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }
          */


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
