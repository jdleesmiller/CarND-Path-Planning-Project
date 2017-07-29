#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <array>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/LU"

/**
 * 1D trajectory for Frenet s as a quintic polynomial.
 */
struct Trajectory {
  typedef std::array<double, 6> Coefficients;

  struct JerkMinimizer {
    JerkMinimizer(double t);
    Trajectory operator()(
      double s0, double v0, double a0, double s1, double v1, double a1) const;
  private:
    double t;
    Eigen::FullPivLU<Eigen::Matrix<double, 3, 3> > a;
  };

  Trajectory(double c0, double c1, double c2, double c3, double c4, double c5);

  Trajectory(const Coefficients &coefficients);

  double GetPosition(double t) const;

  double GetSpeed(double t) const;

private:
  Coefficients coefficients;
};

#endif
