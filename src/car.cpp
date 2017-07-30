#include <iostream>

#include "car.hpp"
#include "common.hpp"

// Maximum jerk and acceleration constraints. Set these lower than the limits
// enforced by the simulator, because these are in (s, d) space, and I think
// the simulator is working in (x, y) space. There can therefore be some
// additional acceleration and jerk due to the road geometry, which we don't
// capture here.
const double MAX_JERK = 6; // m/s^3
const double MAX_ACCELERATION = 5; // m/s^2

// Speed limits: the simulator complains if we go over 50mph. Note that this is
// speed in s only --- speed in (x, y) space can be different, so don't put
// this too close to the actual limit.
const double MAX_SPEED = 22; // 22 m/s is just under 50mph;
const double TARGET_SPEED = 20.5; // m/s
const double MIN_SPEED = -2; // m/s

// Check for collisions with other vehicles in (s, d) space. Because this is
// (s, d) space, and because it has to work at high speed, we need large safety
// margins around the vehicle.
const double COLLISION_LENGTH = 18; // m
const double COLLISION_WIDTH = 3.4; // m

// Define lanes. This assumes that there are always 3 lanes.
const double MAX_D = 12; // m
const double LANE_WIDTH = 4; // m

// Main weights for cost function.
const double FAIL_WEIGHT = 50;
const double COLLISION_WEIGHT = 100; // note: multiplied with FAIL_WEIGHT
const double TARGET_SPEED_WEIGHT = 10;
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

void Car::WrapSIfNeeded() {
  if (s.GetPosition(0) > MAX_S) {
    s.TranslateTo(s.GetPosition(0) - MAX_S);
  }
}

double Car::GetS(double t) const {
  return s.GetPosition(t);
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

double Car::GetCost(double t, const std::vector<Car> &other_cars) const {
  double collision_penalty =
    CollidesWithAny(t, other_cars) ? COLLISION_WEIGHT : 0;

  double jerk = s.GetJerk(t);
  double max_jerk_penalty = fmax(0, jerk - MAX_JERK);
  double min_jerk_penalty = -fmin(0, jerk + MAX_JERK);

  double lateral_jerk = d.GetJerk(t);
  double max_lateral_jerk_penalty = fmax(0, lateral_jerk - MAX_JERK);
  double min_lateral_jerk_penalty = -fmin(0, lateral_jerk + MAX_JERK);

  double acceleration = s.GetAcceleration(t);
  double max_accel_penalty = fmax(0, acceleration - MAX_ACCELERATION);
  double min_accel_penalty = -fmin(0, acceleration + MAX_ACCELERATION);

  double lateral_acceleration = d.GetAcceleration(t);
  double max_lateral_accel_penalty =
    fmax(0, lateral_acceleration - MAX_ACCELERATION);
  double min_lateral_accel_penalty =
    -fmin(0, lateral_acceleration + MAX_ACCELERATION);

  double speed = s.GetSpeed(t);
  double max_speed_penalty = fmax(0, speed - MAX_SPEED);
  double min_speed_penalty = -fmin(0, speed - MIN_SPEED);

  double lane_d = d.GetPosition(t);
  double max_d_penalty = fmax(0, lane_d - MAX_D + LANE_WIDTH / 2);
  double min_d_penalty = -fmin(0, lane_d - LANE_WIDTH / 2);

  double fail_penalty = collision_penalty +
    max_jerk_penalty + min_jerk_penalty +
    max_lateral_jerk_penalty + min_lateral_jerk_penalty +
    max_accel_penalty + min_accel_penalty +
    max_lateral_accel_penalty + min_lateral_accel_penalty +
    max_speed_penalty + min_speed_penalty +
    max_d_penalty + min_d_penalty;

  int lane_number = round((lane_d - LANE_WIDTH / 2) / LANE_WIDTH);
  double lane_centre = LANE_WIDTH * (lane_number + 0.5);
  double lane_keeping_penalty =
    LANE_WIDTH * logistic(fabs(lane_d - lane_centre));

  // Other things being equal, prefer to hog the center lane, and avoid the
  // outside lane, because it seems to give occasional spurious 'out of lane'
  // errors.
  double lane_preference_penalty = 2;
  switch (lane_number) {
    case 0:
    lane_preference_penalty = 0.5;
    break;
    case 1:
    lane_preference_penalty = 0;
    break;
  }

  double target_penalty = TARGET_SPEED_WEIGHT * fabs(speed - TARGET_SPEED) +
    fabs(jerk * DT) + fabs(acceleration * DT) +
    fabs(lateral_jerk * DT) + fabs(lateral_acceleration * DT) +
    lane_keeping_penalty + lane_preference_penalty;

  return FAIL_WEIGHT * fail_penalty + TARGET_WEIGHT * target_penalty;
}

double Car::GetTotalCost(
  double t0, double dt, double t1, const std::vector<Car> &other_cars) const
{
  // Find the car's bounding box for collision detection pruning.
  double car_min_s, car_max_s;
  double car_min_d, car_max_d;
  s.GetBounds(t0, dt, t1, car_min_s, car_max_s);
  d.GetBounds(t0, dt, t1, car_min_d, car_max_d);
  car_min_s -= COLLISION_LENGTH;
  car_max_s += COLLISION_LENGTH;
  car_min_d -= COLLISION_WIDTH;
  car_max_d += COLLISION_WIDTH;

  // Prune the list of other cars based on bounding boxes.
  std::vector<Car> close_cars;
  close_cars.reserve(other_cars.size());
  for (auto it = other_cars.cbegin(); it != other_cars.cend(); ++it) {
    double other_min_s, other_max_s;
    double other_min_d, other_max_d;
    it->s.GetFirstOrderBounds(t0, t1, other_min_s, other_max_s);
    it->d.GetFirstOrderBounds(t0, t1, other_min_d, other_max_d);

    bool can_collide = BoxesIntersect(
      car_min_d, car_max_d, car_min_s, car_max_s,
      other_min_d, other_max_d, other_min_s, other_max_s
    );
    if (can_collide) close_cars.push_back(*it);
  }

  double cost = 0;
  for (double t = t0; t <= t1; t += dt) {
    cost += GetCost(t, close_cars) * dt;
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
  return fabs(ds) < COLLISION_LENGTH && fabs(dd) < COLLISION_WIDTH;
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
