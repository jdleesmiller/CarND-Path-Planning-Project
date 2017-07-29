#include <iostream>

#include "car.hpp"

const double MAX_S = 6945.554;

const double DT = 0.02;

const double MAX_JERK = 6; // m/s^3
const double MAX_ACCELERATION = 6; // m/s^2
const double MAX_SPEED = 22; // 22 m/s is just under 50mph;
const double TARGET_SPEED = 20.5; // m/s
const double MIN_SPEED = -2; // m/s
const double COLLISION_LENGTH = 12; // m
const double COLLISION_WIDTH = 3.7; // m
const double MAX_D = 12; // m
const double LANE_WIDTH = 4;

const double FAIL_WEIGHT = 50;
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
  double s0, double v0, double a0, double d0, double dv0, double da0,
  double s1, double v1, double a1, double d1) {
  Trajectory s = jerk_minimizer(s0, v0, a0, s1, v1, a1);
  Trajectory d = jerk_minimizer(d0, dv0, da0, d1, 0, 0);
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

double Car::GetDSpeed(double t) const {
  return d.GetSpeed(t);
}

double Car::GetDAcceleration(double t) const {
  return d.GetAcceleration(t);
}

double logistic(double x) {
  return 1.0 / (1.0 + exp(-x));
}

double Car::GetCost(double t, const std::vector<Car> &other_cars, bool debug) const {
  double collision_penalty =
    CollidesWithAny(t, other_cars) ? 10 : 0;

  double jerk = s.GetJerk(t) * DT;
  double max_jerk_penalty = fmax(0, jerk - MAX_JERK);
  double min_jerk_penalty = -fmin(0, jerk + MAX_JERK);

  double acceleration = s.GetAcceleration(t) * DT;
  double max_accel_penalty = fmax(0, acceleration - MAX_ACCELERATION);
  double min_accel_penalty = -fmin(0, acceleration + MAX_ACCELERATION);

  double speed = s.GetSpeed(t);
  double max_speed_penalty = fmax(0, speed - MAX_SPEED);
  double min_speed_penalty = -fmin(0, speed - MIN_SPEED);

  double lane_d = d.GetPosition(t);
  double max_d_penalty = fmax(0, lane_d - MAX_D + LANE_WIDTH / 2);
  double min_d_penalty = -fmin(0, lane_d - LANE_WIDTH / 2);

  double fail_penalty = collision_penalty +
    max_jerk_penalty + min_jerk_penalty +
    max_accel_penalty + min_accel_penalty +
    max_speed_penalty + min_speed_penalty +
    max_d_penalty + min_d_penalty;

  double lane_centre = LANE_WIDTH * (
    round((lane_d - LANE_WIDTH / 2) / LANE_WIDTH) + 0.5);
  double lane_keeping_penalty = 5 * logistic(fabs(lane_d - lane_centre));
  double lateral_jerk = d.GetJerk(t);
  double lateral_acceleration = d.GetAcceleration(t);

  double target_penalty = fabs(speed - TARGET_SPEED) +
    fabs(jerk * DT) + fabs(acceleration * DT) +
    fabs(lateral_jerk * DT) + fabs(lateral_acceleration * DT) +
    lane_keeping_penalty;

  if (debug) {
    std::cout << "fail=" << fail_penalty << " target=" << target_penalty << std::endl;
    std::cout << "collision=" << collision_penalty <<
      " jerk =" << jerk <<
      " max_jerk_penalty =" << max_jerk_penalty <<
      " min_jerk_penalty =" << min_jerk_penalty <<
      " acceleration =" << acceleration <<
      " max_accel_penalty =" << max_accel_penalty <<
      " min_accel_penalty =" << min_accel_penalty <<
      " speed =" << speed <<
      " max_speed_penalty =" << max_speed_penalty <<
      " min_speed_penalty =" << min_speed_penalty <<
      " max_d_penalty =" << max_d_penalty <<
      " min_d_penalty =" << max_d_penalty <<
      std::endl;
    std::cout <<
      " target_speed_penalty =" << fabs(speed - TARGET_SPEED) <<
      " jerk_penalty =" << fabs(jerk) * DT <<
      " acceleration_penalty =" << fabs(acceleration) * DT <<
      " lane_keeping_penalty =" << lane_keeping_penalty <<
      std::endl;
  }

  return FAIL_WEIGHT * fail_penalty + TARGET_WEIGHT * target_penalty;
}

double Car::GetTotalCost(
  double t0, double dt, double t1, const std::vector<Car> &other_cars, bool debug) const
{
  double cost = 0;
  for (double t = t0; t <= t1; t += dt) {
    cost += GetCost(t, other_cars, debug) * dt;
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
  if (fabs(ds) < COLLISION_LENGTH && fabs(dd) < COLLISION_WIDTH) {
    // std::cout << "COLLISION: " << car << " at " << t << " ds=" << ds << " dd=" << dd << std::endl;
    return true;
  }
  return false;
}

bool Car::CollidesWithAny(double t, const std::vector<Car> &other_cars) const {
  for (auto it = other_cars.cbegin(); it != other_cars.cend(); ++it) {
    if (Collides(t, *it)) return true;
  }
  return false;
}

std::ostream &operator<<(std::ostream &os, const Car &car) {
  os << "s:" << car.s << " d:" << car.d;
  return os;
}
