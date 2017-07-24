#include "map.hpp"
#include "spline.h"

#include <cmath>
#include <fstream>
#include <limits>
#include <iostream>
#include <sstream>
#include <string>

// The max s value before wrapping around the track back to 0
// double max_s = 6945.554;

Map::Map(const char *file) {
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

  if (waypoints.size() == 0) {
    std::cerr << "No waypoint data found (wrong pwd?)" << std::endl;
    exit(-1);
  }
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

Map::CartesianPoint Map::GetCartesian(double s, double d) const {
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


  // vector<MapWaypoint> map_waypoints;
  // for (size_t i = 0; i < map_waypoints_x.size(); ++i) {
  //   map_waypoints.push_back(
  //     MapWaypoint(map_waypoints_x[i], map_waypoints_y[i]));
  // }
  // sort(map_waypoints.begin(), map_waypoints.end(), MapWaypoint::ByX());
  // vector<double> map_waypoints_x_sorted;
  // vector<double> map_waypoints_y_sorted;
  // for (size_t i = 0; i < map_waypoints.size(); ++i) {
  //   cout << map_waypoints[i].x << endl;
  //   map_waypoints_x_sorted.push_back(map_waypoints[i].x);
  //   map_waypoints_y_sorted.push_back(map_waypoints[i].y);
  // }

  // tk::spline map_waypoints_xy;
  // map_waypoints_xy.set_points(map_waypoints_x_sorted, map_waypoints_y_sorted);
