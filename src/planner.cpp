#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>

#include "planner.hpp"
#include "trajectory.hpp"

const double DT = 0.02; // seconds per timestep
const double HORIZON = 3; // seconds
const double LATENCY = 0.3; // seconds

const size_t NUM_SAMPLES = 2000;

std::random_device rd;
std::mt19937 gen(rd());

const double S_STDEV = 50;
const double V_STDEV = 10;
const double A_STDEV = 10;

std::normal_distribution<> S_DISTRIBUTION(0, S_STDEV);
std::normal_distribution<> V_DISTRIBUTION(0, V_STDEV);
std::normal_distribution<> A_DISTRIBUTION(0, A_STDEV);

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

  double end_time = elapsed_time + HORIZON - LATENCY;
  double end_car_s = car.GetS(end_time);
  double end_car_v = car.GetSpeed(end_time);
  double end_car_a = car.GetAcceleration(end_time);
  std::cout << "END s=" << end_car_s << " v=" << end_car_v << " a=" << end_car_a << std::endl;
  // double new_car_d = car.GetD(end_time);
  Car best_car;
  double best_cost = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < NUM_SAMPLES; ++i) {
    double candidate_s = end_car_s + S_DISTRIBUTION(gen);
    double candidate_v = end_car_v + V_DISTRIBUTION(gen);
    double candidate_a = end_car_a + A_DISTRIBUTION(gen);
    Car candidate_car = Car::MakeQuintic(jerk_minimizer,
      car_s, car_v, car_a, 6.16483,
      candidate_s, candidate_v, candidate_a, 6.16483);
    double candidate_cost = candidate_car.GetTotalCost(
      0, DT, HORIZON - LATENCY, other_cars);
    if (i % 100 == 0) {
      std::cout << "s=" << candidate_s << " v=" << candidate_v << " a=" << candidate_a << " cost=" << candidate_cost << std::endl;
      std::cout << candidate_car << std::endl;
    }
    if (candidate_cost < best_cost) {
      best_cost = candidate_cost;
      best_car = candidate_car;
    }
  }
  std::cout << best_car << " best cost=" << best_cost << std::endl;
  car = best_car;

  TrimPlan();

  for (double t = DT; t <= HORIZON - LATENCY; t += DT) {
    double s_t = car.GetS(t);
    Map::CartesianPoint point = map.GetCartesianSpline(s_t, 6.16483);
    plan_x.push_back(point.x);
    plan_y.push_back(point.y);
    plan_s.push_back(s_t);
    plan_d.push_back(6.16483);
  }

  // for (size_t i = 0; i < GetPlanSize(); ++i) {
  //   std::cout << i << "\t" << plan_s[i] << "\t" << plan_x[i] << "\t" << plan_y[i] << "\t";
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
