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

size_t Map::GetClosestWaypointIndex(double x, double y) const {
	double closest_dist = std::numeric_limits<double>::infinity();
	size_t closest_index = 0;

	for (size_t i = 0; i < waypoints.size(); i++) {
		double dist = distance(x, y, waypoints[i].x, waypoints[i].y);
		if (dist < closest_dist) {
			closest_dist = dist;
			closest_index = i;
		}
	}

	return closest_index;
}

size_t Map::GetNextWaypointIndex(double x, double y, double theta) const {
	size_t closest_index = GetClosestWaypointIndex(x, y);
	double map_x = waypoints[closest_index].x;
	double map_y = waypoints[closest_index].y;

	double heading = atan2(map_y - y, map_x - x);
	double angle = fabs(theta - heading);

	if (angle > M_PI / 4) {
		closest_index++;
	}

	return closest_index;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
Map::FrenetPoint Map::GetFrenet(double x, double y, double theta) const {
	size_t next_wp = GetNextWaypointIndex(x, y, theta);
	size_t prev_wp;

	if (next_wp > 0) {
    prev_wp = next_wp - 1;
  } else {
		prev_wp = waypoints.size() - 1;
  }

	double n_x = waypoints[next_wp].x - waypoints[prev_wp].x;
	double n_y = waypoints[next_wp].y - waypoints[prev_wp].y;
	double x_x = x - waypoints[prev_wp].x;
	double x_y = y - waypoints[prev_wp].y;

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

  FrenetPoint frenet;

	frenet.d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - waypoints[prev_wp].x;
	double center_y = 2000 - waypoints[prev_wp].y;
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if (centerToPos <= centerToRef) {
		frenet.d *= -1;
	}

	// calculate s value
	frenet.s = 0;
	for (size_t i = 0; i < prev_wp; ++i) {
		frenet.s += distance(
      waypoints[i].x, waypoints[i].y,
      waypoints[i+1].x, waypoints[i+1].y);
	}
	frenet.s += distance(0, 0, proj_x, proj_y);

  return frenet;
}

Map::CartesianPoint Map::GetCartesianLinear(double s, double d) const {
  int prev_wp = -1;
  while(s > waypoints[prev_wp + 1].s && prev_wp < (int)(waypoints.size() - 1)) {
    prev_wp++;
  }
  int wp = (prev_wp + 1) % waypoints.size();

  double heading = atan2(
    waypoints[wp].y - waypoints[prev_wp].y,
    waypoints[wp].x - waypoints[prev_wp].x);

  // the x,y,s along the segment
  double seg_s = s - waypoints[prev_wp].s;
  double seg_x = waypoints[prev_wp].x + seg_s * cos(heading);
  double seg_y = waypoints[prev_wp].y + seg_s * sin(heading);

  double perp_heading = heading - M_PI/2;

  CartesianPoint point;
  point.x = seg_x + d*cos(perp_heading);
  point.y = seg_y + d*sin(perp_heading);

  return point;
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
