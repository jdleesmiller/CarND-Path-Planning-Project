#include "common.hpp"

// The max s value before wrapping around the track back to 0
const double MAX_S = 6945.554;

// Simulator timestep, in seconds.
const double DT = 0.02;

// Based on https://gamedev.stackexchange.com/questions/586/what-is-the-fastest-way-to-work-out-2d-bounding-box-intersection
bool BoxesIntersect(double x00, double x10, double y00, double y10,
  double x01, double x11, double y01, double y11)
{
  return !(x01 > x10 || x11 < x00 || y11 < y00 || y01 > y10);
}
