#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include "planner.hpp"
#include "trajectory.hpp"

const double DT = 0.02; // seconds per timestep
const double HORIZON = 5; // seconds
const double LATENCY = 0.3; // seconds

const double LANE_WIDTH = 4;

Planner::Planner(const Map &map) :
  map(map), jerk_minimizer(HORIZON - LATENCY) { }

size_t Planner::GetPlanSize() const {
  return plan_x.size();
}

void Planner::ClearOtherCars() {
  other_cars.clear();
}

void Planner::AddOtherCar(double s, double d, double vx, double vy) {
  other_cars.push_back(Car::MakeLinear(s, d, vx, vy));
}

double Planner::GetCost(const Car &c) const {
  return c.GetTotalCost(0, DT, HORIZON - LATENCY, other_cars);
}

void Planner::Update(size_t previous_plan_size,
  double car_s, double car_d, double car_v)
{
  double car_a = 0;
  double elapsed_time = AdvancePlan(previous_plan_size);
  if (elapsed_time == 0) {
    car = Car::MakeInitial(jerk_minimizer, car_s, car_v, car_a, car_d);
  } else {
    car_s = car.GetS(elapsed_time);
    car_v = car.GetSpeed(elapsed_time);
    car_a = car.GetAcceleration(elapsed_time);
    car_d = car.GetD(elapsed_time);
  }

  double end_time = HORIZON - LATENCY;
  std::array<double, 3> goal;
  goal[0] = car.GetS(end_time);
  goal[1] = car.GetSpeed(end_time);
  goal[2] = car.GetAcceleration(end_time);

  std::array<double, 3> step_size;
  step_size[0] = 5; // s coordinate
  step_size[1] = 1;
  step_size[2] = 1;

  double delta_size = 1.2;
  std::array<double, 5> deltas;
  deltas[0] = -delta_size;
  deltas[1] = -1.0 / delta_size;
  deltas[2] = 0;
  deltas[3] = 1.0 / delta_size;
  deltas[4] = delta_size;

  std::array<int, 3> lane_deltas({{-1, 0, 1}});
  std::array<Car, 3> lane_cars({{car, car, car}});
  size_t best_lane = 1;
  double best_lane_cost = std::numeric_limits<double>::infinity();

  for (size_t lane = 0; lane < lane_deltas.size(); ++lane) {
    double lane_d = LANE_WIDTH * (
      round((car_d - LANE_WIDTH / 2) / LANE_WIDTH) + lane_deltas[lane] + 0.5);

    double end_cost;
    for (;;) {
      double start_cost = GetCost(lane_cars[lane]);
      for (size_t i = 0; i < goal.size(); ++i) {
        size_t best_delta = 0;
        double best_cost = std::numeric_limits<double>::infinity();
        for (size_t j = 0; j < deltas.size(); ++j) {
          goal[i] += step_size[i] * deltas[j];
          Car candidate_car = Car::MakeQuintic(jerk_minimizer,
            car_s, car_v, car_a, car_d,
            goal[0], goal[1], goal[2], lane_d);
          goal[i] -= step_size[i] * deltas[j];
          double candidate_cost = GetCost(candidate_car);
          if (candidate_cost < best_cost) {
            best_cost = candidate_cost;
            best_delta = j;
          }
        }
        if (deltas[best_delta] == 0) {
          step_size[i] /= delta_size;
        } else {
          goal[i] += step_size[i] * deltas[best_delta];
          step_size[i] *= deltas[best_delta];
        }
      }
      lane_cars[lane] = Car::MakeQuintic(jerk_minimizer,
        car_s, car_v, car_a, car_d,
        goal[0], goal[1], goal[2], lane_d);
      end_cost = GetCost(lane_cars[lane]);
      // std:: cout << "GD " << lane_d << " " << goal[0] << "\t" << goal[1] << "\t" << goal[2] << "\t" << end_cost << "\t" << car << std::endl;
      if (fabs(start_cost - end_cost) < 1e-3) break;
    }

    if (end_cost < best_lane_cost) {
      best_lane_cost = end_cost;
      best_lane = lane;
    }
  }
  // std::cout << lane_deltas[best_lane] << std::endl;
  car = lane_cars[best_lane];

  TrimPlan();

  for (double t = DT; t <= HORIZON - LATENCY; t += DT) {
    double s_t = car.GetS(t);
    double d_t = car.GetD(t);
    Map::CartesianPoint point = map.GetCartesianSpline(s_t, d_t);
    plan_x.push_back(point.x);
    plan_y.push_back(point.y);
    plan_s.push_back(s_t);
    plan_d.push_back(d_t);
  }

  // for (size_t i = 0; i < GetPlanSize(); ++i) {
  //   std::cout << i << "\t" << plan_s[i] << "\t" << plan_x[i] << "\t" << plan_y[i] << "\t" << plan_d[i] << "\t";
  //   if (i > 0) {
  //     double vx = (plan_x[i] - plan_x[i - 1]) / DT;
  //     double vy = (plan_y[i] - plan_y[i - 1]) / DT;
  //     std::cout << sqrt(vx * vx + vy * vy);
  //   }
  //   std::cout << std::endl;
  // }
}

double Planner::AdvancePlan(size_t previous_plan_size) {
  size_t n;
  if (previous_plan_size > GetPlanSize()) {
    std::cerr << "WARNING: previous plan too large" << std::endl;
    return 0;
  } else {
    n = GetPlanSize() - previous_plan_size;
  }
  plan_x.erase(plan_x.begin(), plan_x.begin() + n);
  plan_y.erase(plan_y.begin(), plan_y.begin() + n);
  plan_s.erase(plan_s.begin(), plan_s.begin() + n);
  plan_d.erase(plan_d.begin(), plan_d.begin() + n);
  return n * DT;
}

void Planner::TrimPlan() {
  size_t n = std::min((size_t)(LATENCY / DT), plan_x.size());
  plan_x.erase(plan_x.begin() + n, plan_x.end());
  plan_y.erase(plan_y.begin() + n, plan_y.end());
  plan_s.erase(plan_s.begin() + n, plan_s.end());
  plan_d.erase(plan_d.begin() + n, plan_d.end());
}
