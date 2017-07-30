#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include <vector>

#include "trajectory.hpp"

struct Car {
  Car();

  Car(const Trajectory &s, const Trajectory &d);

  static Car MakeInitial(
    const Trajectory::JerkMinimizer &jerk_minimizer,
    double s0, double v0, double a0, double d0);

  static Car MakeQuintic(
    const Trajectory::JerkMinimizer &jerk_minimizer,
    double s0, double v0, double a0, double d0, double dv0, double da0,
    double s1, double v1, double a1, double d1);

  static Car MakeLinear(double s0, double d0, double vx, double vy);

  void WrapSIfNeeded();

  double GetS(double t) const;

  double GetSpeed(double t) const;

  double GetAcceleration(double t) const;

  double GetJerk(double t) const;

  double GetD(double t) const;

  double GetDSpeed(double t) const;

  double GetDAcceleration(double t) const;

  double GetCost(double t, const std::vector<Car> &other_cars, bool debug = false) const;

  double GetTotalCost(double t0, double dt, double t1,
    const std::vector<Car> &other_cars, bool debug = false) const;

  bool Collides(double t, const Car &car) const;

  bool CollidesWithAny(double t, const std::vector<Car> &other_cars) const;

  Trajectory s;
  Trajectory d;
};

std::ostream &operator<<(std::ostream &os, const Car &car);

#endif
