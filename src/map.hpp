#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

#include <vector>

#include "spline.h"

/**
 * Map from Frenet (s, d) space to Cartesian (x, y) space using the waypoints.
 * See README for more information.
 */
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

  std::vector<Waypoint> waypoints;
  tk::spline x_spline;
  tk::spline y_spline;
  tk::spline d_x_spline;
  tk::spline d_y_spline;
};

#endif
