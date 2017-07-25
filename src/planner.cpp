#include <algorithm>
#include <cmath>
#include <iostream>

#include "planner.hpp"

const double SPEED = 21; // 22 m/s is just under 50mph
const double DT = 0.02; // seconds per timestep
const double HORIZON = 3; // seconds
const double LATENCY = 0.3; // seconds

const double UPDATE_TOLERANCE = 1e-3;

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

void Planner::Plan(double car_s, double car_d) {
  double new_car_s;

  TrimPlan();

  if (PlanIsEmpty()) {
    new_car_s = car_s;
  } else {
    new_car_s = plan_s.back();
  }

  for (double t = 0; t < HORIZON - LATENCY; t += DT) {
    new_car_s += DT * SPEED;
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
