#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <deque>

#include "map.hpp"

struct Planner {
  Planner(const Map &map);

  void Update(double previous_x, double previous_y);

  void Plan(double car_s, double car_d);

  bool PlanIsEmpty() const;

  const std::deque<double> &GetPlanX() const { return plan_x; }

  const std::deque<double> &GetPlanY() const { return plan_y; }

private:
  void TrimPlan();

  const Map &map;
  std::deque<double> plan_x;
  std::deque<double> plan_y;
  std::deque<double> plan_s;
  std::deque<double> plan_d;
};

#endif
