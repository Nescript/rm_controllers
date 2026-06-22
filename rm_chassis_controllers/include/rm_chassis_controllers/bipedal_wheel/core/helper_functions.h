#pragma once

#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

#include "bipedal_wheel/core/core_types.h"
#include "bipedal_wheel/core/dynamics/gen_A.h"
#include "bipedal_wheel/core/dynamics/gen_B.h"

namespace bipedal_wheel_core
{

static inline double get_LM(double l)
{
  return 0.218 * l + 0.075;
}

static inline double get_i_p(double l)
{
  return 0.4 * l + 0.07;
}

static inline double get_theta_leg_offset(double l)
{
  (void)l;
  return M_PI_4 / 2.0;
}

inline void generateAB(const LqrModelParams& model_params,
                       Eigen::Matrix<double, STATE_DIM, STATE_DIM>& a,
                       Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>& b,
                       double leg_length)
{
  double A[36] = { 0. }, B[12] = { 0. };
  double Lm = get_LM(leg_length);
  double L = leg_length - Lm;
  double i_p = get_i_p(leg_length);

  gen_A(model_params.i_m, i_p, model_params.i_w, L, Lm, model_params.M, model_params.r, model_params.g,
        model_params.l, model_params.m_p, model_params.m_w, A);
  gen_B(model_params.i_m, i_p, model_params.i_w, L, Lm, model_params.M, model_params.r, model_params.l,
        model_params.m_p, model_params.m_w, B);

  // Map 1D arrays to Eigen Matrices A and B
  // clang-format off
  a << 0.  , 1.0, 0.0, 0.0, 0.0 , 0.0,
       A[1], 0.0, 0.0, 0.0, A[25], 0.0,
       0.0 , 0.0, 0.0, 1.0, 0.0 , 0.0,
       A[3], 0.0, 0.0, 0.0, A[27], 0.0,
       0.0 , 0.0, 0.0, 0.0, 0.0 , 1.0,
       A[5], 0.0, 0.0, 0.0, A[29], 0.0;
  b << 0.0 , 0.0,
       B[1], B[7],
       0.0 , 0.0,
       B[3], B[9],
       0.0 , 0.0,
       B[5], B[11];
  // clang-format on
}

inline double shortest_angular_distance(double from, double to)
{
  double angle = to - from;
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

inline void clamp(double& val, double minVal, double maxVal)
{
  if (val < minVal)
    val = minVal;
  else if (val > maxVal)
    val = maxVal;
}

inline double f_spring_force(double L0, double l1, double l2, const SpringParams& spring)
{
  double cos_theta3 = (l1 * l1 + l2 * l2 - L0 * L0) / (2.0 * l1 * l2);
  clamp(cos_theta3, -1.0, 1.0); // Prevent NaN in std::acos
  double theta3 = std::acos(cos_theta3);
  double ls = std::sqrt(spring.s2 * spring.s2 + spring.s3 * spring.s3 - 2.0 * spring.s2 * spring.s3 * std::cos(theta3 - spring.alpha_s));
  double sin_theta3 = std::sin(theta3);
  if (ls == 0.0 || sin_theta3 == 0.0) return 0.0;
  double Fv = spring.f_spring * (L0 * spring.s2 * spring.s3 * std::sin(theta3 - spring.alpha_s)) / (ls * l1 * l2 * sin_theta3);
  return Fv;
}

class LinearKalmanFilter2D
{
public:
  LinearKalmanFilter2D()
  {
    x_.setZero();
    P_.setIdentity();
    Q_ << 1.0, 0.0,
          0.0, 1.0;
    R_ << 200.0, 0.0,
          0.0, 200.0;
  }

  void init(double initial_velocity)
  {
    x_ << initial_velocity, 0.0;
    P_.setIdentity();
  }

  void predict(double dt)
  {
    Eigen::Matrix2d A;
    A << 1.0, dt,
         0.0, 1.0;
    x_ = A * x_;
    P_ = A * P_ * A.transpose() + Q_;
  }

  void update(const Eigen::Vector2d& z, const Eigen::Matrix2d& R)
  {
    Eigen::Matrix2d H = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d S = H * P_ * H.transpose() + R;
    Eigen::Matrix2d K = P_ * H.transpose() * S.inverse();
    x_ = x_ + K * (z - H * x_);
    P_ = (Eigen::Matrix2d::Identity() - K * H) * P_;
  }

  const Eigen::Vector2d& getState() const { return x_; }
  void setState(const Eigen::Vector2d& x) { x_ = x; }

private:
  Eigen::Vector2d x_;
  Eigen::Matrix2d P_;
  Eigen::Matrix2d Q_;
  Eigen::Matrix2d R_;
};

}  // namespace bipedal_wheel_core
