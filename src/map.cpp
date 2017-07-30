#include <cmath>
#include <fstream>
#include <limits>
#include <iostream>
#include <sstream>
#include <string>

#include "map.hpp"
#include "common.hpp"

// Repat this many points at the start and the end to make sure the
// splines knit together when s wraps around MAX_S.
const size_t SPLINE_OVERLAP = 4;

Map::Map(const char *file) {
  LoadWaypoints(file);
  FitSplines();
}

double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

void Map::LoadWaypoints(const char *file) {
  std::ifstream in(file, std::ifstream::in);
  std::string line;
  while (getline(in, line)) {
  	std::istringstream iss(line);
    Waypoint waypoint;
  	iss >> waypoint.x;
  	iss >> waypoint.y;
  	iss >> waypoint.s;
  	iss >> waypoint.d_x;
  	iss >> waypoint.d_y;
    waypoints.push_back(waypoint);
  }

  if (waypoints.size() < SPLINE_OVERLAP) {
    std::cerr << "Not enough waypoints (wrong pwd?)" << std::endl;
    exit(-1);
  }
}

void Map::FitSplines() {
  std::vector<double> s;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> d_x;
  std::vector<double> d_y;

  // Append some points at the end with negative s chainage.
  size_t n = waypoints.size();
  for (size_t i = n - SPLINE_OVERLAP; i < n; ++i) {
    s.push_back(waypoints[i].s - MAX_S);
    x.push_back(waypoints[i].x);
    y.push_back(waypoints[i].y);
    d_x.push_back(waypoints[i].d_x);
    d_y.push_back(waypoints[i].d_y);
  }

  // Now add the waypoints.
  for (size_t i = 0; i < n; ++i) {
    s.push_back(waypoints[i].s);
    x.push_back(waypoints[i].x);
    y.push_back(waypoints[i].y);
    d_x.push_back(waypoints[i].d_x);
    d_y.push_back(waypoints[i].d_y);
  }

  // Append some of the waypoints at the start, after the maximum
  // actual s chainage, to help the spline close.
  for (size_t i = 0; i < SPLINE_OVERLAP; ++i) {
    s.push_back(waypoints[i].s + MAX_S);
    x.push_back(waypoints[i].x);
    y.push_back(waypoints[i].y);
    d_x.push_back(waypoints[i].d_x);
    d_y.push_back(waypoints[i].d_y);
  }

  // for (size_t i = 0; i < s.size(); ++i) {
  //   std::cout << s[i] << "\t" << x[i] << "\t" << y[i] << std::endl;
  // }

  x_spline.set_points(s, x);
  y_spline.set_points(s, y);
  d_x_spline.set_points(s, d_x);
  d_y_spline.set_points(s, d_y);
}

Map::CartesianPoint Map::GetCartesianSpline(double s, double d) const {
  double x_s = x_spline(s);
  double y_s = y_spline(s);
  double d_x_s = d_x_spline(s);
  double d_y_s = d_y_spline(s);

  // The spline smoothing may not preserve the norm, so renormalize.
  double d_norm = sqrt(d_x_s * d_x_s + d_y_s * d_y_s);

  // std::cout << "s=" << s << " d=" << d <<
  //   " x_s=" << x_s <<
  //   " y_s=" << y_s <<
  //   " d_x_s=" << d_x_s <<
  //   " d_y_s=" << d_y_s <<
  //   " d_norm=" << d_norm << std::endl;

  CartesianPoint point;
  point.x = x_s + d * d_x_s / d_norm;
  point.y = y_s + d * d_y_s / d_norm;
  return point;
}
