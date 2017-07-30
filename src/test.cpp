#include <cassert>
#include <iostream>

#include "test.hpp"
#include "common.hpp"
#include "trajectory.hpp"

using namespace std;

const double EPSILON = 1e-9;

void test_box_intersection() {
  assert(BoxesIntersect(
    1, 3, 1, 3,
    2, 4, 2, 4
  ));
  assert(!BoxesIntersect(
    1, 3, 1, 3,
    4, 5, 4, 5
  ));
}

void test_trajectory_first_order_bounds_1() {
  Trajectory trajectory(1, 2, 0, 0, 0, 0);
  assert(trajectory.GetPosition(1) == 3);
  assert(trajectory.GetPosition(10) == 21);

  double min_value, max_value;
  trajectory.GetFirstOrderBounds(1, 10, min_value, max_value);
  assert(min_value == 3);
  assert(max_value == 21);
}

void test_trajectory_first_order_bounds_2() {
  Trajectory trajectory(1, -2, 0, 0, 0, 0);
  assert(trajectory.GetPosition(1) == -1);
  assert(trajectory.GetPosition(10) == -19);

  double min_value, max_value;
  trajectory.GetFirstOrderBounds(1, 10, min_value, max_value);
  assert(min_value == -19);
  assert(max_value == -1);
}

void test_trajectory_bounds_1() {
  Trajectory trajectory(1, 2, -0.5, 0, 0, 0);
  assert(trajectory.GetPosition(1) == 2.5);
  assert(trajectory.GetPosition(2) == 3);
  assert(trajectory.GetPosition(10) == -29);

  double min_value, max_value;
  trajectory.GetBounds(1, 0.1, 10, min_value, max_value);
  assert(fabs(min_value - -29) < EPSILON);
  assert(fabs(max_value - 3) < EPSILON);
}

void run_tests() {
  test_box_intersection();
  test_trajectory_first_order_bounds_1();
  test_trajectory_first_order_bounds_2();
  test_trajectory_bounds_1();
}
