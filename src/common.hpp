#ifndef PATH_PLANNING_COMMON_H
#define PATH_PLANNING_COMMON_H

// The max s value before wrapping around the track back to 0
extern const double MAX_S;

// Simulator timestep, in seconds.
extern const double DT;

// Check whether bounding boxes intersect.
bool BoxesIntersect(double x00, double x10, double y00, double y10,
  double x01, double x11, double y01, double y11);

#endif
