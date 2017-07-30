#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <chrono>
#include <deque>

#include "car.hpp"
#include "map.hpp"

/**
 * Main trajectory planner. See README for more information.
 *
 * @param map
 */
struct Planner {
  Planner(const Map &map);

  void ClearOtherCars();

  void AddOtherCar(double s, double d, double vx, double vy);

  void Update(size_t previous_plan_size,
    double car_s, double car_d, double car_v);

  size_t GetPlanSize() const;

  const std::deque<double> &GetPlanX() const { return plan_x; }

  const std::deque<double> &GetPlanY() const { return plan_y; }

private:
  double AdvancePlan(size_t previous_plan_size);

  void TrimPlan();

  double GetCost(const Car &c) const;

  const Map &map;

  std::deque<double> plan_x;
  std::deque<double> plan_y;

  Trajectory::JerkMinimizer jerk_minimizer;

  // The current s and d trajectories recommended by the planner.
  Car car;

  // Other cars from sensor fusion data (to be avoided).
  std::vector<Car> other_cars;

  // Time of last update, if any.
  std::chrono::steady_clock::time_point t;

  // Smoothed time from last update to current update, in seconds.
  double latency;
};

#endif
