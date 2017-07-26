#include <algorithm>
#include <cmath>
#include <iostream>

#include "planner.hpp"

const double MAX_S = 6945.554;

const double TARGET_SPEED = 21; // 22 m/s is just under 50mph
const double DT = 0.02; // seconds per timestep
const double HORIZON = 3; // seconds
const double LATENCY = 0.3; // seconds

const double UPDATE_TOLERANCE = 1e-3;

const double LANE_WIDTH = 4.0; // meters
const double LANE_TOLERANCE = 0.5; // meters

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

bool Planner::PlanIsEmpty() const {
  return plan_x.empty();
}

void Planner::Update(double previous_x, double previous_y) {
  while (!PlanIsEmpty()) {
    double dx = plan_x.front() - previous_x;
    double dy = plan_y.front() - previous_y;
    if (fabs(dx) < UPDATE_TOLERANCE && fabs(dy) < UPDATE_TOLERANCE) {
      return;
    }
    plan_x.pop_front();
    plan_y.pop_front();
    plan_s.pop_front();
    plan_d.pop_front();
  }
  std::cerr << "WARNING: failed to locate previous (x, y)" << std::endl;
}

void Planner::ClearOtherCars() {
  other_cars.clear();
}

void Planner::AddOtherCar(const OtherCar &other_car) {
  other_cars.push_back(other_car);
}

void Planner::Plan(double car_s, double car_d, double car_speed) {
  double new_car_s;

  TrimPlan();

  if (PlanIsEmpty()) {
    new_car_s = car_s;
  } else {
    new_car_s = plan_s.back();
    if (new_car_s > MAX_S) {
      new_car_s -= MAX_S;
    }
  }

  double target_speed = TARGET_SPEED;
  size_t blocking_index = FindNearestBlockingCar(new_car_s, car_d);
  if (blocking_index < other_cars.size()) {
    double headway = other_cars[blocking_index].GetRange(car_s) / car_speed;
    if (headway <= HORIZON) {
      std::cout << "BLOCKING " << other_cars[blocking_index].id << " " << other_cars[blocking_index].d << " v=" << other_cars[blocking_index].GetSpeed() << std::endl;
      target_speed = other_cars[blocking_index].GetSpeed() * 0.99;
    }
  }

  for (double t = 0; t < HORIZON - LATENCY; t += DT) {
    new_car_s += DT * target_speed;
    Map::CartesianPoint point = map.GetCartesianSpline(new_car_s, 6.16483);
    plan_x.push_back(point.x);
    plan_y.push_back(point.y);
    plan_s.push_back(new_car_s);
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
