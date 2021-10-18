#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include <tuple>
#include "spline.h"

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

  int lane = 1; // self's lane
  double ref_vel = 0.0; // reference relocity in mph

  h.onMessage([&lane, &ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          double car_yaw = j[1]["yaw"];   // in degree
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"].get<vector<vector<double>>>();
          
          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          // double dist_inc = 0.5;
          // double next_s;
          // double next_d = 6;
          // vector<double> xy;
          // for (int i = 1; i <= 50; i++) {
          //   double next_s = car_s + i * dist_inc;
          //   xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          //   next_x_vals.push_back(xy[0]);
          //   next_y_vals.push_back(xy[1]);
          // }
          
          int prev_size = previous_path_x.size();
          if (prev_size > 0) {
            // projects the car's s into the "future" based on previously leftover trajectory
            // car_s is used to be compared to other car's "future" s to determine the gap, which lead to behavior planning
            car_s = end_path_s;   
          }

          // get the states of surroundings
          vector<bool> surrounding = surrounding_states(sensor_fusion, prev_size, lane, car_s);
          // std::cout << "front car too close" << surrounding[0] << std::endl;
          // std::cout << "left car too close" << surrounding[1] << std::endl;
          // std::cout << "right car too close" << surrounding[2] << std::endl;

          double vel_inc;
          // update the lane number and the acceleration value based on surroundings
          std::tie(lane, vel_inc) = update_lane_and_accl(lane, ref_vel, surrounding);
          // std::cout << "new lane:" << lane << " acceleration:" << accl << std::endl;

          // determine reference anchors and the previous point (for making the path tangent to the car)
          double ref_yaw, ref_x, ref_y;
          double ref_x_prev, ref_y_prev;
          if (prev_size < 2) {
            // use self's localization data
            ref_yaw = deg2rad(car_yaw); // use the current car yaw
            ref_x = car_x;    // initial anchor points for spline generation
            ref_y = car_y;
            ref_x_prev = ref_x - cos(ref_yaw);  // artificial points
            ref_y_prev = ref_y - sin(ref_yaw);
          } else {
            // use the end of the previous leftover trajectory
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            ref_x_prev = previous_path_x[prev_size - 2];
            ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);  // use the "future" car yaw
          }
          // std::cout << "ref_yaw: " << ref_yaw << std::endl;
                    
          // setting up future spline anchor points based on future lane
          int num_of_future_anchors = 3;
          double s_interval = 30.0; // 30m per anchor
          vector<vector<double>> future_anchors;
          for (int i = 1; i <= num_of_future_anchors; i++) {
            future_anchors.push_back(
              getXY(car_s + i * s_interval, W/2 + W * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y)
            );
          }

          // combine all spline anchors
          vector<double> anchors_x{ref_x_prev, ref_x};
          vector<double> anchors_y{ref_y_prev, ref_y};
          for (int i = 0; i < num_of_future_anchors; i++) {
            anchors_x.push_back(future_anchors[i][0]);
            anchors_y.push_back(future_anchors[i][1]);
          }
          
          // convert spline anchors to local coordinates, with origin at (ref_x, ref_y)
          for (int i = 0; i < anchors_x.size(); i++) {
            double shift_x = anchors_x[i] - ref_x;
            double shift_y = anchors_y[i] - ref_y;
            anchors_x[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
            anchors_y[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
          }

          // create spline 
          tk::spline spl;
          spl.set_points(anchors_x, anchors_y);


          // building new path based on previously left over path, (0 <= prev_size < NUM_OF_PATH_POINTS)
          for (int i = 0; i < prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          } 

          // tricky part to understand
          double target_x = 30.0;
          double target_y = spl(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0;

          for (int i = 1; i <= NUM_OF_PATH_POINTS - prev_size; i++) {
            // working in local coordinates
            ref_vel += vel_inc;
            double N = target_dist / (timestep * ref_vel / 2.24); // number of segments in the hypotenuse
            double x_point = x_add_on + target_x / N;
            double y_point = spl(x_point);

            x_add_on = x_point;

            // convert to map coordinates
            double x0 = x_point;
            double y0 = y_point; 

            x_point = ref_x + x0 * cos(ref_yaw) - y0 * sin(ref_yaw);
            y_point = ref_y + x0 * sin(ref_yaw) + y0 * cos(ref_yaw);

            // add to the path trajectory
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