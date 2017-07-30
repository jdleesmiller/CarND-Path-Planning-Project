#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

#include <vector>

#include "spline.h"

struct Map {
  Map(const char *file);

  struct Waypoint {
    double x;
    double y;
    double s;
    double d_x;
    double d_y;
  };

  struct CartesianPoint {
    double x;
    double y;
  };

  // Transform from Frenet s,d coordinates to Cartesian x,y using the spline.
  CartesianPoint GetCartesianSpline(double s, double d) const;

private:
  void LoadWaypoints(const char *file);

  void FitSplines();

  size_t GetClosestWaypointIndex(double x, double y) const;

  size_t GetNextWaypointIndex(double x, double y, double theta) const;

  std::vector<Waypoint> waypoints;
  tk::spline x_spline;
  tk::spline y_spline;
  tk::spline d_x_spline;
  tk::spline d_y_spline;
};

#endif
