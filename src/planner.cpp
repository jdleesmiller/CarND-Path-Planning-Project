#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>

#include "planner.hpp"
#include "trajectory.hpp"

const double DT = 0.02; // seconds per timestep
const double HORIZON = 2.5; // seconds
const double LATENCY = 0.5; // seconds

const size_t NUM_SAMPLES = 12;

std::random_device rd;
std::mt19937 gen(rd());

const double S_STDEV = 3;
const double V_STDEV = 1;
const double A_STDEV = 1;

std::normal_distribution<> S_DISTRIBUTION(0, S_STDEV);
std::normal_distribution<> V_DISTRIBUTION(0, V_STDEV);
std::normal_distribution<> A_DISTRIBUTION(0, A_STDEV);
std::uniform_int_distribution<> D_DISTRIBUTION(0, 2);

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

double Planner::GetCost(const Car &c, bool debug) const {
  return c.GetTotalCost(0, DT, HORIZON - LATENCY, other_cars, debug);
}

// double RoundToLane(double d) {
//   double lane = round((d - LANE_WIDTH / 2) / LANE_WIDTH);
//   return LANE_WIDTH * (lane + 0.5);
// }

void Planner::Update(size_t previous_plan_size,
  double car_s, double car_d, double car_v)
{
  double car_a = 0;
  double car_dv = 0;
  double car_da = 0;
  double elapsed_time = AdvancePlan(previous_plan_size);
  if (elapsed_time == 0) {
    car = Car::MakeInitial(jerk_minimizer, car_s, car_v, car_a, car_d);
  } else {
    car_s = car.GetS(elapsed_time);
    car_v = car.GetSpeed(elapsed_time);
    car_a = car.GetAcceleration(elapsed_time);
    car_d = car.GetD(elapsed_time);
    car_dv = car.GetDSpeed(elapsed_time);
    car_da = car.GetDAcceleration(elapsed_time);
  }

  double end_time = HORIZON - LATENCY;
  double end_s = car.GetS(end_time);
  double end_v = car.GetSpeed(end_time);
  double end_a = car.GetAcceleration(end_time);

  Car best_car;
  double best_cost = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < NUM_SAMPLES; ++i) {
    std::array<double, 3> goal;
    goal[0] = end_s + S_DISTRIBUTION(gen);
    goal[1] = end_v + V_DISTRIBUTION(gen);
    goal[2] = end_a + A_DISTRIBUTION(gen);
    double goal_d = D_DISTRIBUTION(gen) * LANE_WIDTH + LANE_WIDTH / 2;

    std::array<double, 3> step_size;
    step_size[0] = 5; // s coordinate
    step_size[1] = 1;
    step_size[2] = 1;

    double delta_size = 1;
    std::array<double, 3> deltas;
    deltas[0] = -delta_size;
    deltas[1] = 0;
    deltas[2] = delta_size;

    car = Car::MakeQuintic(jerk_minimizer,
      car_s, car_v, car_a, car_d, car_dv, car_da,
      goal[0], goal[1], goal[2], goal_d);

    for (;;) {
      double start_cost = GetCost(car);
      for (size_t i = 0; i < goal.size(); ++i) {
        size_t best_delta = 0;
        double best_cost = std::numeric_limits<double>::infinity();
        for (size_t j = 0; j < deltas.size(); ++j) {
          goal[i] += step_size[i] * deltas[j];
          Car candidate_car = Car::MakeQuintic(jerk_minimizer,
            car_s, car_v, car_a, car_d, car_dv, car_da,
            goal[0], goal[1], goal[2], goal_d);
          double candidate_cost = GetCost(candidate_car);
          // std::cout << "GD " << i << " " << j << " " << goal[0] << "\t" << goal[1] << "\t" << goal[2] << "\t" << goal_d << "\t" << candidate_cost << "\t" << std::endl;
          if (candidate_cost < best_cost) {
            best_cost = candidate_cost;
            best_delta = j;
          }
          goal[i] -= step_size[i] * deltas[j];
        }
        if (deltas[best_delta] == 0) {
          step_size[i] /= delta_size;
        } else {
          goal[i] += step_size[i] * deltas[best_delta];
          step_size[i] *= deltas[best_delta];
        }
      }

      car = Car::MakeQuintic(jerk_minimizer,
        car_s, car_v, car_a, car_d, car_dv, car_da,
        goal[0], goal[1], goal[2], goal_d);
      double end_cost = GetCost(car, false);
      if (end_cost < best_cost) {
        best_cost = end_cost;
        best_car = car;
      }
      // std::cout << "GD END " << goal[0] << "\t" << goal[1] << "\t" << goal[2] << "\t" << goal_d << "\t" << end_cost << "\t" << std::endl;
      if (fabs(start_cost - end_cost) < 1e-3) break;
    }
  }

  std::cout << best_car << " best cost=" << best_cost << std::endl;
  car = best_car;

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
