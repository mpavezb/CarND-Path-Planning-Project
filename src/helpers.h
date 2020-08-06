#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>

#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "data_types.h"

// for convenience
using std::string;
using std::vector;

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
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, std::shared_ptr<udacity::Map> map) {
  double closestLen = 100000;  // large number
  int closestWaypoint = 0;

  for (int i = 0; i < map->waypoints_x.size(); ++i) {
    double map_x = map->waypoints_x[i];
    double map_y = map->waypoints_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta,
                 std::shared_ptr<udacity::Map> map) {
  int closestWaypoint = ClosestWaypoint(x, y, map);

  double map_x = map->waypoints_x[closestWaypoint];
  double map_y = map->waypoints_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = std::min(2 * pi() - angle, angle);

  if (angle > pi() / 2) {
    ++closestWaypoint;
    if (closestWaypoint == map->waypoints_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
udacity::Frenet getFrenet(double x, double y, double theta,
                          std::shared_ptr<udacity::Map> map) {
  int next_wp = NextWaypoint(x, y, theta, map);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = map->waypoints_x.size() - 1;
  }

  double n_x = map->waypoints_x[next_wp] - map->waypoints_x[prev_wp];
  double n_y = map->waypoints_y[next_wp] - map->waypoints_y[prev_wp];
  double x_x = x - map->waypoints_x[prev_wp];
  double x_y = y - map->waypoints_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  // see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - map->waypoints_x[prev_wp];
  double center_y = 2000 - map->waypoints_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(map->waypoints_x[i], map->waypoints_y[i],
                         map->waypoints_x[i + 1], map->waypoints_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
udacity::Point getXY(double s, double d, std::shared_ptr<udacity::Map> map) {
  int prev_wp = -1;

  while (s > map->waypoints_s[prev_wp + 1] &&
         (prev_wp < (int)(map->waypoints_s.size() - 1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp + 1) % map->waypoints_x.size();

  double heading = atan2((map->waypoints_y[wp2] - map->waypoints_y[prev_wp]),
                         (map->waypoints_x[wp2] - map->waypoints_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - map->waypoints_s[prev_wp]);

  double seg_x = map->waypoints_x[prev_wp] + seg_s * cos(heading);
  double seg_y = map->waypoints_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

std::vector<double> getPathX(const udacity::Path &path) {
  std::vector<double> result;
  for (const auto &point : path) {
    result.push_back(point.x);
  }
  return result;
}

std::vector<double> getPathY(const udacity::Path &path) {
  std::vector<double> result;
  for (const auto &point : path) {
    result.push_back(point.y);
  }
  return result;
}

std::uint8_t getLaneIdFromFrenet(double d, double lane_width) {
  return fmax(fmin(2, floor(d / lane_width)), 0);
}

void printPath(const udacity::Path &path, const std::string &name) {
  std::stringstream ss;
  ss << name << "[";
  for (int i = 0; i < path.size() - 1; ++i) {
    ss << path[i].x << ", ";
  }
  if (path.size() > 0) {
    ss << path[path.size() - 1].x;
  }
  ss << "]";
  std::cout << ss.str() << std::endl;
}

#endif  // HELPERS_H
