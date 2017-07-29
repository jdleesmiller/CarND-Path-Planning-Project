#include <iostream>

#include "car.hpp"

const double MAX_S = 6945.554;

const double MAX_JERK = 10; // m/s^3
const double MAX_ACCELERATION = 10; // m/s^2
const double MAX_SPEED = 22; // 22 m/s is just under 50mph;
const double TARGET_SPEED = 20; // m/s
const double MIN_SPEED = -2; // m/s
const double COLLISION_RADIUS_SQUARED = 4; // m

const double FAIL_WEIGHT = 10;
const double TARGET_WEIGHT = 1;

Car::Car() { }

Car::Car(const Trajectory &s, const Trajectory &d) : s(s), d(d) { }

Car Car::MakeInitial(
  const Trajectory::JerkMinimizer &jerk_minimizer,
  double s0, double v0, double a0, double d0) {
  Trajectory s(s0, v0, a0/2, 0.1, 0, 0);
  Trajectory d(d0, 0, 0, 0, 0, 0);
  return Car(s, d);
}

Car Car::MakeQuintic(
  const Trajectory::JerkMinimizer &jerk_minimizer,
  double s0, double v0, double a0, double d0,
  double s1, double v1, double a1, double d1) {
  Trajectory s = jerk_minimizer(s0, v0, a0, s1, v1, a1);
  Trajectory d = jerk_minimizer(d0, 0, 0, d1, 0, 0);
  return Car(s, d);
}

Car Car::MakeLinear(double s0, double d0, double vx, double vy) {
  double speed = sqrt(vx * vx + vy * vy);
  Trajectory s(s0, speed, 0, 0, 0, 0);
  Trajectory d(d0, 0, 0, 0, 0, 0);
  return Car(s, d);
}

double Car::GetS(double t) const {
  double s_t = s.GetPosition(t);
  if (s_t > MAX_S) {
    s_t -= MAX_S;
  }
  return s_t;
}

double Car::GetSpeed(double t) const {
  return s.GetSpeed(t);
}

double Car::GetAcceleration(double t) const {
  return s.GetAcceleration(t);
}

double Car::GetJerk(double t) const {
  return s.GetJerk(t);
}

double Car::GetD(double t) const {
  return d.GetPosition(t);
}

double logistic(double x) {
  return 1.0 / (1.0 + exp(-x));
}

double Car::GetCost(double t, const std::vector<Car> &other_cars) const {
  double collision_penalty = CollidesWithAny(t, other_cars);

  double jerk = s.GetJerk(t);
  double max_jerk_penalty = fmax(0, jerk - MAX_JERK);
  double min_jerk_penalty = -fmin(0, jerk + MAX_JERK);

  double acceleration = s.GetAcceleration(t);
  double max_accel_penalty = fmax(0, acceleration - MAX_ACCELERATION);
  double min_accel_penalty = -fmin(0, acceleration + MAX_ACCELERATION);

  double speed = s.GetSpeed(t);
  double max_speed_penalty = fmax(0, speed - MAX_SPEED);
  double min_speed_penalty = -fmin(0, speed + MIN_SPEED);
  double target_speed_penalty = logistic(fabs(speed - TARGET_SPEED));

  double fail_penalty = collision_penalty +
    max_jerk_penalty + min_jerk_penalty +
    max_accel_penalty + min_accel_penalty +
    max_speed_penalty + min_speed_penalty;
  return FAIL_WEIGHT * fail_penalty + TARGET_WEIGHT * target_speed_penalty;
}

double Car::GetTotalCost(
  double t0, double dt, double t1, const std::vector<Car> &other_cars) const
{
  double cost = 0;
  for (double t = t0; t <= t1; t += dt) {
    cost += GetCost(t, other_cars) * dt;
  }
  return cost;
}

double Range(double s0, double s1) {
  double ds = s1 - s0;
  if (ds > MAX_S / 2) {
    // The other car is near the end of the track; our car is near the start.
    ds -= MAX_S;
  } else if (ds < -MAX_S / 2) {
    // The other car is near the start of the track; our car is near the end.
    ds += MAX_S;
  }
  return ds;
}

bool Car::Collides(double t, const Car &car) const {
  double ds = Range(car.GetS(t), GetS(t));
  double dd = car.GetD(t) - GetD(t);
  return ds * ds + dd * dd < COLLISION_RADIUS_SQUARED;
}

bool Car::CollidesWithAny(double t, const std::vector<Car> &other_cars) const {
  for (auto it = other_cars.cbegin(); it != other_cars.cend(); ++it) {
    if (Collides(t, *it)) return true;
  }
  return false;
}

std::ostream &operator<<(std::ostream &os, const Car &car) {
  os << "s:" << car.s;
  return os;
}
