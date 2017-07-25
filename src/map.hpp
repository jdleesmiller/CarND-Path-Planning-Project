#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

#include <vector>

struct Map {
  Map(const char *file);

  struct Waypoint {
    double x;
    double y;
    double s;
    double d_x;
    double d_y;
  };

  struct FrenetPoint {
    double s;
    double d;
  };

  struct CartesianPoint {
    double x;
    double y;
  };

  // Transform from Frenet s,d coordinates to Cartesian x,y
  CartesianPoint GetCartesian(double s, double d) const;

  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  FrenetPoint GetFrenet(double x, double y, double theta) const;

private:
  void LoadWaypoints(const char *file);

  size_t GetClosestWaypointIndex(double x, double y) const;

  size_t GetNextWaypointIndex(double x, double y, double theta) const;

  std::vector<Waypoint> waypoints;
};

#endif
