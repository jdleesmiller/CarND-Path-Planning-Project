#include "trajectory.hpp"

Trajectory::Trajectory() : coefficients({{0, 0, 0, 0, 0, 0}}) {}

Trajectory::Trajectory(
  double c0, double c1, double c2, double c3, double c4, double c5) :
  coefficients({{c0, c1, c2, c3, c4, c5}}) {}

Trajectory::Trajectory(const Coefficients &coefficients) :
  coefficients(coefficients) {}

Trajectory::JerkMinimizer::JerkMinimizer(double t) : t(t) {
  double t2 = t*t;
  double t3 = t*t2;
  double t4 = t2*t2;
  double t5 = t*t4;
  Eigen::Matrix<double, 3, 3> weights;
  weights << t3, t4, t5,
    3*t2, 4*t3, 5*t4,
    6*t, 12*t2, 20*t3;
  a = weights.fullPivLu();
}

Trajectory Trajectory::JerkMinimizer::operator()(
  double s0, double v0, double a0, double s1, double v1, double a1) const {

  Eigen::Vector3d b;
  b << s1 - (s0 + v0 * t + 0.5 * a0 * t * t),
       v1 - (v0 + a0 * t),
       a1 - a0;

  Eigen::Vector3d x;
  x = a.solve(b);

  return Trajectory(s0, v0, a0/2, x[0], x[1], x[2]);
}

double Trajectory::GetPosition(double t) const {
  return coefficients[0] + t * (
    coefficients[1] + t * (
      coefficients[2] + t * (
        coefficients[3] + t * (
          coefficients[4] + t * (
            coefficients[5]
          )
        )
      )
    )
  );
}

double Trajectory::GetSpeed(double t) const {
  return coefficients[1] + t * (
    2 * coefficients[2] + t * (
      3 * coefficients[3] + t * (
        4 * coefficients[4] + t * (
          5 * coefficients[5]
        )
      )
    )
  );
}

double Trajectory::GetAcceleration(double t) const {
  return 2 * coefficients[2] + t * (
    6 * coefficients[3] + t * (
      12 * coefficients[4] + t * (
        20 * coefficients[5]
      )
    )
  );
}

double Trajectory::GetJerk(double t) const {
  return 6 * coefficients[3] + t * (
    24 * coefficients[4] + t * (
      60 * coefficients[5]
    )
  );
}

std::ostream &operator<<(std::ostream &os, const Trajectory &trajectory) {
  for (auto it = trajectory.GetCoefficients().cbegin();
    it != trajectory.GetCoefficients().cend(); ++it) {
    os << " " << *it;
  }
  return os;
}
