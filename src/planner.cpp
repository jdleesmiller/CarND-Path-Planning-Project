#include <algorithm>
#include <cmath>
#include <iostream>

#include "planner.hpp"
#include "trajectory.hpp"

const double MAX_S = 6945.554;

const double DT = 0.02; // seconds per timestep
const double HORIZON = 3; // seconds
const double LATENCY = 0.3; // seconds

const double LANE_WIDTH = 4.0; // meters
const double LANE_TOLERANCE = 0.5; // meters

const double MAX_JERK = 2; // m/s^3
const double MAX_ACCELERATION = 2; // m/s^2
const double MAX_SPEED = 21; // 22 m/s is just under 50mph;
const double MIN_SPEED = -5; // m/s

//
// Assuming we operate at maximum jerk (or negative jerk) for the whole horizon,
// find the final speed and position that would result. This gives us an
// 'envelope'
//
void GetMaxDistanceAndSpeed(
  double sign, double max_speed, double &s, double &v)
{
  double a = 0;
  for (double t = 0; t < HORIZON - LATENCY; t += DT) {
    a = a + sign * MAX_JERK * DT;
    if (a > MAX_ACCELERATION) { a = MAX_ACCELERATION; }
    if (a < -MAX_ACCELERATION) { a = -MAX_ACCELERATION; }
    v = v + a * DT;
    if (v > max_speed) { v = max_speed; }
    if (v < MIN_SPEED) { v = MIN_SPEED; }
    s = s + v * DT;
  }
}

double Planner::OtherCar::GetSpeed() const {
  return sqrt(vx * vx + vy * vy);
}

double Planner::OtherCar::GetRange(double car_s) const {
  double ds = s - car_s;
  if (ds > MAX_S / 2) {
    // The other car is near the end of the track; our car is near the start.
    ds -= MAX_S;
  } else if (ds < -MAX_S / 2) {
    // The other car is near the start of the track; our car is near the end.
    ds += MAX_S;
  }
  return ds;
}

bool Planner::OtherCar::IsBlocking(double car_s, double car_d) const {
  return GetRange(car_s) >= 0 &&
    fabs(d - car_d) < LANE_WIDTH / 2 + LANE_TOLERANCE;
}

Planner::Planner(const Map &map) : map(map) { }

size_t Planner::GetPlanSize() const {
  return plan_x.size();
}

void Planner::Update(size_t previous_plan_size) {
  size_t n;
  if (previous_plan_size > GetPlanSize()) {
    std::cerr << "WARNING: previous plan too large" << std::endl;
    return;
  } else {
    n = GetPlanSize() - previous_plan_size;
  }
  plan_x.erase(plan_x.begin(), plan_x.begin() + n);
  plan_y.erase(plan_y.begin(), plan_y.begin() + n);
  plan_s.erase(plan_s.begin(), plan_s.begin() + n);
  plan_d.erase(plan_d.begin(), plan_d.begin() + n);
}

void Planner::ClearOtherCars() {
  other_cars.clear();
}

void Planner::AddOtherCar(const OtherCar &other_car) {
  other_cars.push_back(other_car);
}

void Planner::Plan(double car_s, double car_d, double car_speed) {
  // idea:
  // search in 3 (s, s_dot) planes, one for each of d = -1, 0 1.
  // must finish the movement within the horizon (zero accel, zero jerk)
  // - not great for getting up to speed --- would actually want to
  //   finish with positive acceleration, unless the horizon is long enough to
  //   reach maximum speed
  // upper bound for s_dot:
  // - assume maximum jerk, up to maximum acceleration, up to max speed.
  // upper bound for s:
  // - just integrate the max s_dot profile
  //

  TrimPlan();

  if (GetPlanSize() > 1) {
    car_s = plan_s.back();
    if (car_s > MAX_S) {
      car_s -= MAX_S;
    }
    car_speed = (plan_s[GetPlanSize() - 1] - plan_s[GetPlanSize() - 2]) / DT;
  }

  double target_speed = MAX_SPEED;
  size_t blocking_index = FindNearestBlockingCar(car_s, car_d);
  if (blocking_index < other_cars.size()) {
    double headway = other_cars[blocking_index].GetRange(car_s) / car_speed;
    if (headway <= HORIZON) {
      std::cout << "BLOCKING " << other_cars[blocking_index].id << " " << other_cars[blocking_index].d << " v=" << other_cars[blocking_index].GetSpeed() << std::endl;
      target_speed = other_cars[blocking_index].GetSpeed() * 0.99;
    }
  }

  double final_s = car_s;
  double final_v = car_speed;
  GetMaxDistanceAndSpeed(1, target_speed, final_s, final_v);
  std::cout << "init s=" << car_s << "init v=" << car_speed << " final s=" << final_s << " final v=" << final_v << std::endl;

  Trajectory::JerkMinimizer jerk_minimizer(HORIZON - LATENCY);
  Trajectory trajectory(jerk_minimizer(
    car_s, car_speed, 0, final_s, final_v, 0));

  for (size_t i = 0; i < plan_x.size(); ++i) {
    std::cout << i << "\t" << plan_s[i] << "\t" << plan_x[i] << "\t" << plan_y[i] << std::endl;
  }

  for (double t = 0; t < HORIZON - LATENCY; t += DT) {
    double s_t = trajectory.GetPosition(t);
    Map::CartesianPoint point = map.GetCartesianSpline(s_t, 6.16483);
    std::cout << t << "\t" << s_t << "\t" << point.x << "\t" << point.y << "\t" << trajectory.GetSpeed(t) << std::endl;
    plan_x.push_back(point.x);
    plan_y.push_back(point.y);
    plan_s.push_back(s_t);
    plan_d.push_back(6.16483);
  }
}

void Planner::TrimPlan() {
  size_t n = std::min((size_t)(LATENCY / DT), plan_x.size());
  plan_x.erase(plan_x.begin() + n, plan_x.end());
  plan_y.erase(plan_y.begin() + n, plan_y.end());
  plan_s.erase(plan_s.begin() + n, plan_s.end());
  plan_d.erase(plan_d.begin() + n, plan_d.end());
}

size_t Planner::FindNearestBlockingCar(double car_s, double car_d) {
  double min_s = std::numeric_limits<double>::infinity();
  size_t min_i = std::numeric_limits<size_t>::max();
  for (size_t i = 0; i < other_cars.size(); ++i) {
    if (other_cars[i].IsBlocking(car_s, car_d)) {
      if (other_cars[i].s < min_s) {
        min_s = other_cars[i].s;
        min_i = i;
      }
    }
  }
  return min_i;
}
