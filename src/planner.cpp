#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>

#include "planner.hpp"
#include "common.hpp"
#include "trajectory.hpp"

// Initial latency estimate, before we start estimating it.
const double LATENCY_DEFAULT = 0.25; // seconds

// Smoothing factor for the exponential moving average of the timestep.
const double LATENCY_SMOOTH = 0.1;

const double HORIZON = 3.25; // seconds
const double LATENCY = 0.25; // seconds
const size_t MAX_ITERS = 10;

const size_t NUM_SAMPLES = 20;

std::random_device rd;
std::mt19937 gen(rd());

const double S_STDEV = 10;
const double V_STDEV = 2;
const double A_STDEV = 1;

std::normal_distribution<> S_DISTRIBUTION(0, S_STDEV);
std::normal_distribution<> V_DISTRIBUTION(0, V_STDEV);
std::normal_distribution<> A_DISTRIBUTION(0, A_STDEV);

const double LANE_WIDTH = 4;

Planner::Planner(const Map &map) :
  map(map), jerk_minimizer(HORIZON - LATENCY),
  t(std::chrono::steady_clock::now()),
  latency(LATENCY_DEFAULT) { }

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
  // Find out where the car is in (s, d) space, based on how many of the
  // points we sent in the last timestep have come back as "previous points"
  // in this timestep --- this is the elapsed time (in whole timesteps) since
  // our last update.
  double car_a = 0;
  double car_dv = 0;
  double car_da = 0;
  double elapsed_time = AdvancePlan(previous_plan_size);
  if (elapsed_time == 0) {
    car = Car::MakeInitial(jerk_minimizer, car_s, car_v, car_a, car_d);
  } else {
    car.WrapSIfNeeded();
    car_s = car.GetS(elapsed_time);
    car_v = car.GetSpeed(elapsed_time);
    car_a = car.GetAcceleration(elapsed_time);
    car_d = car.GetD(elapsed_time);
    car_dv = car.GetDSpeed(elapsed_time);
    car_da = car.GetDAcceleration(elapsed_time);
  }

  // Latency compensation: project where the car will be at the end of the
  // latency delay, based on its previous trajectory.
  double end_time = HORIZON - LATENCY;
  double end_s = car.GetS(end_time);
  double end_v = car.GetSpeed(end_time);
  double end_a = car.GetAcceleration(end_time);

  Car best_car;
  double best_cost = std::numeric_limits<double>::infinity();

  // Check each possible goal lane.
  for (int lane = 0; lane < 3; ++lane) {
    double goal_d = lane * LANE_WIDTH + LANE_WIDTH / 2;

    // For each goal lane, generate goal samples.
    for (size_t num_samples = 0; num_samples < NUM_SAMPLES; ++num_samples) {
      std::array<double, 3> goal;
      goal[0] = end_s + S_DISTRIBUTION(gen);
      goal[1] = end_v + V_DISTRIBUTION(gen);
      goal[2] = end_a + A_DISTRIBUTION(gen);

      std::array<double, 3> step_size;
      step_size[0] = 5; // s coordinate
      step_size[1] = 2;
      step_size[2] = 1;

      double delta_size = 1;
      std::array<double, 3> deltas;
      deltas[0] = -delta_size;
      deltas[1] = 0;
      deltas[2] = delta_size;

      car = Car::MakeQuintic(jerk_minimizer,
        car_s, car_v, car_a, car_d, car_dv, car_da,
        goal[0], goal[1], goal[2], goal_d);

      // For each sample, try to improve the plan by hill climbing (well,
      // we're minimizing, so in that sense we are going down hill).
      for (size_t num_iters = 0; num_iters < MAX_ITERS; ++num_iters) {
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
        double end_cost = GetCost(car);
        if (end_cost < best_cost) {
          best_cost = end_cost;
          best_car = car;
        }
        // std::cout << "GD END " << goal[0] << "\t" << goal[1] << "\t" << goal[2] << "\t" << goal_d << "\t" << end_cost << "\t" << std::endl;
        if (fabs(start_cost - end_cost) < 1e-3) break;
      }
    }
  }

  car = best_car;

  // Convert the reference trajectory into (x, y) space for the simulator.
  TrimPlan();
  for (double t = DT; t <= HORIZON - LATENCY; t += DT) {
    double s_t = car.GetS(t);
    double d_t = car.GetD(t);
    Map::CartesianPoint point = map.GetCartesianSpline(s_t, d_t);
    plan_x.push_back(point.x);
    plan_y.push_back(point.y);
  }

  // Keep track of latency for monitoring.
  auto new_t = std::chrono::steady_clock::now();
  std::chrono::duration<double> dt_duration = new_t - t;
  double new_latency = dt_duration.count();
  latency = new_latency * LATENCY_SMOOTH + latency * (1 - LATENCY_SMOOTH);
  t = new_t;

  std::cout <<
    "s=" << best_car.GetS(0) <<
    " latency=" << latency <<
    " cost=" << best_cost << std::endl;

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
  return n * DT;
}

void Planner::TrimPlan() {
  size_t n = std::min((size_t)(LATENCY / DT), plan_x.size());
  plan_x.erase(plan_x.begin() + n, plan_x.end());
  plan_y.erase(plan_y.begin() + n, plan_y.end());
}
