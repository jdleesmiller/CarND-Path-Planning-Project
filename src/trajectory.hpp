#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <array>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/LU"

/**
 * 1D quintic polynomial trajectory.
 */
struct Trajectory {
  typedef std::array<double, 6> Coefficients;

  struct JerkMinimizer {
    JerkMinimizer(double t);
    Trajectory operator()(
      double s0, double v0, double a0, double s1, double v1, double a1) const;
    double GetTime() const { return t; }
  private:
    double t;
    Eigen::FullPivLU<Eigen::Matrix<double, 3, 3> > a;
  };

  Trajectory();

  Trajectory(double c0, double c1, double c2, double c3, double c4, double c5);

  Trajectory(const Coefficients &coefficients);

  const Coefficients &GetCoefficients() const { return coefficients; }

  double GetPosition(double t) const;

  void TranslateTo(double c0);

  double GetSpeed(double t) const;

  double GetAcceleration(double t) const;

  double GetJerk(double t) const;

  /**
   * Get the min and max value by checking the qunitic at each time point in
   * [t0, t1] by dt.
   *
   * @param t0 start time (inclusive)
   * @param dt time step
   * @param t1 end time (inclusive)
   * @param min_value set to the minimum value for the trajectory in [t0, t1]
   * @param max_value set to the maximum value for the trajectory in [t0, t1]
   */
  void GetBounds(double t0, double dt, double t1,
    double &min_value, double &max_value) const;

  /**
   * Get the min and max value assuming that the quintic is affine. That is,
   * we only consider the first two coefficients and ignore the rest.
   *
   * @param t0 start time (inclusive)
   * @param t1 end time (inclusive)
   * @param min_value set to the minimum value for the trajectory in [t0, t1]
   * @param max_value set to the maximum value for the trajectory in [t0, t1]
   */
  void GetFirstOrderBounds(double t0, double t1,
    double &min_value, double &max_value) const;

private:
  Coefficients coefficients;
};

std::ostream &operator<<(std::ostream &os, const Trajectory &trajectory);

#endif
