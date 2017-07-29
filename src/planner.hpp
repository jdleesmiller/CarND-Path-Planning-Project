#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <deque>

#include "car.hpp"
#include "map.hpp"

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

  size_t FindNearestBlockingCar(double car_s, double car_d);

  const Map &map;
  std::deque<double> plan_x;
  std::deque<double> plan_y;
  std::deque<double> plan_s;
  std::deque<double> plan_d;
  Trajectory::JerkMinimizer jerk_minimizer;
  Car car;
  std::vector<Car> other_cars;
};

#endif
