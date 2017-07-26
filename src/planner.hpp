#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <deque>

#include "map.hpp"

struct Planner {
  struct OtherCar {
    size_t id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;

    double GetSpeed() const;

    double GetRange(double car_s) const;

    bool IsBlocking(double car_s, double car_d) const;
  };

  Planner(const Map &map);

  void Update(double previous_x, double previous_y);

  void ClearOtherCars();

  void AddOtherCar(const OtherCar &other_car);

  void Plan(double car_s, double car_d, double car_speed);

  bool PlanIsEmpty() const;

  const std::deque<double> &GetPlanX() const { return plan_x; }

  const std::deque<double> &GetPlanY() const { return plan_y; }

private:
  void TrimPlan();

  size_t FindNearestBlockingCar(double car_s, double car_d);

  const Map &map;
  std::deque<double> plan_x;
  std::deque<double> plan_y;
  std::deque<double> plan_s;
  std::deque<double> plan_d;
  std::vector<OtherCar> other_cars;
};

#endif
