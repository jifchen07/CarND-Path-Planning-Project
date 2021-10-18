#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <tuple>

// for convenience
using std::string;
using std::vector;

const double W = 4.0; // lane width
const double SPEED_MAX = 49; // in mph, just a little below the speed limit
const double SPEED_MIN = 10.0;
const int NUM_OF_PATH_POINTS = 50; // total number of path points to be sent to simulator
const int NUM_OF_POINTS_PER_SECOND = 50;
const double timestep = 1.0 / NUM_OF_POINTS_PER_SECOND;
const double ACCEL_MAX = 5.0;  // in m/s^2, less than 10, tunable
const double VEL_INC = ACCEL_MAX * 2.24 / NUM_OF_POINTS_PER_SECOND; // delta(mph) in a time step

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
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

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}


// return the surrounding environment of the car
vector<bool> surrounding_states(const vector<vector<double>> &sensor_fusion,
                                int prev_path_size, int lane_self, double s_self) {                                
  bool front_car_too_close = false;
  bool left_car_too_close = false;
  bool right_car_too_close = false;
  int lane_num;
  double s, d, vx, vy, v;   // surrounding car's position and velocity variables
  double s_diff; int lane_diff;
  for (int i = 0; i < sensor_fusion.size(); i++) {
    s = sensor_fusion[i][5];
    d = sensor_fusion[i][6];

    // determine the lane number of a surrounding car
    if (0 < d && d < W) lane_num = 0;
    else if (W < d && d < 2*W) lane_num = 1;
    else if (2*W < d && d < 3*W) lane_num = 2;
    else continue;

    // calculate car's speed
    vx = sensor_fusion[i][3];
    vy = sensor_fusion[i][4];
    v = sqrt(vx * vx + vy * vy);

    // estimate the car's position at the end of the previous path
    s += ((double) prev_path_size * 0.02 * v);
    
    s_diff = s - s_self;
    lane_diff = lane_num - lane_self;

    if (!front_car_too_close && !lane_diff) {
      front_car_too_close = 0 < s_diff && s_diff < 30;
    } else if (!left_car_too_close && lane_diff == -1) {
      left_car_too_close  = -10 < s_diff && s_diff < 30;
    } else if (!right_car_too_close && lane_diff == 1) {
      right_car_too_close = -10 < s_diff && s_diff < 30;
    }

    if (front_car_too_close && left_car_too_close && right_car_too_close) {
      return {true, true, true};
    }
  }

  return {front_car_too_close, left_car_too_close, right_car_too_close};
}

//
std::tuple<int, double> update_lane_and_accl(int current_lane, double ref_vel, 
                                            const vector<bool> &surrounding) {
  bool front_car_too_close = surrounding[0];
  bool left_car_too_close = surrounding[1];
  bool right_car_too_close = surrounding[2];
  double speed_change = 0.0;  // the acceleration

  if (front_car_too_close) {  // front car is too slow, need to change lane or slow down
    if (current_lane > 0 && !left_car_too_close) {
      current_lane -= 1;  // change to left lane
    } else if (current_lane < 2 && !right_car_too_close) {
      current_lane += 1;  // change to right lane
    } else if (ref_vel > VEL_INC) {
      speed_change = -VEL_INC;  // stay in same lane and reduce speed, while preparing for lane change
    }
  } else {  // front is clear in the current lane
    if (ref_vel < SPEED_MAX - VEL_INC) {
      speed_change = VEL_INC;
    }
    if (current_lane == 0 && !right_car_too_close) {
      current_lane += 1;  // good driving behavior: don't stay in the leftmost lane if not for passing
    }
  }

  return std::make_tuple(current_lane, speed_change);
}


#endif  // HELPERS_H