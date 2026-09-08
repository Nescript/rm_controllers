#include "rm_chassis_controllers/bipedal_wheel_controller/powerID.h"

#include <cmath>

PowerModel::PowerModel()
{
  init(0.999, 0.001);
}

void PowerModel::init(double lambda, double alpha_ridge)
{
  c_vec.setZero();
  A_acc = 1e-3 * Eigen::Matrix<double, 12, 12>::Identity();
  B_acc.setZero();
  lambda_ = lambda;
  alpha_ridge_ = alpha_ridge;
  sample_count_ = 0;
}

void PowerModel::setLambda(double lambda)
{
  lambda_ = lambda;
}

void PowerModel::setAlphaRidge(double alpha_ridge)
{
  alpha_ridge_ = alpha_ridge;
}

const Eigen::Matrix<double, 12, 1>& PowerModel::getC() const
{
  return c_vec;
}

void PowerModel::updateD(int t, double v, double omega, double a, double alpha, double P_meas)
{
  (void)t;  // 预留：暂未使用
  Eigen::Matrix<double, 12, 1> phi_k = extractFeatures(v, omega, a, alpha);
  A_acc = lambda_ * A_acc + phi_k * phi_k.transpose();
  B_acc = lambda_ * B_acc + P_meas * phi_k;
  sample_count_++;
}

bool PowerModel::isExcited(double v, double omega, double a, double alpha) const
{
  // 持续激励（PE）判定阈值：若各运动学量均在静止死区内，则判定为未受激励
  constexpr double v_thres = 0.05;        // m/s
  constexpr double omega_thres = 0.08;    // rad/s
  constexpr double a_thres = 0.15;        // m/s^2
  constexpr double alpha_thres = 0.3;     // rad/s^2

  return (std::abs(v) > v_thres || std::abs(omega) > omega_thres ||
          std::abs(a) > a_thres || std::abs(alpha) > alpha_thres);
}

Eigen::Matrix<double, 12, 1> PowerModel::extractFeatures(double v, double omega, double a, double alpha)
{
  Eigen::Matrix<double, 12, 1> phi_k;
  phi_k << 1.0, v * a, omega * alpha, a * a, alpha * alpha, std::abs(v), std::abs(omega), v * v, omega * omega,
      std::abs(a), std::abs(alpha), std::abs(v * omega);
  return phi_k;
}

bool PowerModel::solve()
{
  if (sample_count_ <= 12)
  {
    return false;
  }
  Eigen::Matrix<double, 12, 12> a_solve = A_acc + alpha_ridge_ * Eigen::Matrix<double, 12, 12>::Identity();
  a_solve = 0.5 * (a_solve + a_solve.transpose());
  Eigen::LDLT<Eigen::Matrix<double, 12, 12>> ldlt(a_solve);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive())
  {
    return false;
  }
  Eigen::Matrix<double, 12, 1> c_raw = ldlt.solve(B_acc);
  if (!c_raw.allFinite())
  {
    return false;
  }

  // 物理合理性校验与约束截断 (Physical bounds & clamping)
  // c[0]: 静态损耗功率 (W)
  // c[1]: 等效质量 M (kg)
  // c[2]: 等效转动惯量 J (kg*m^2)
  // 若基础物理参数严重倒挂（出现负质量或负转动惯量），判定本次求解非法，保留历史估计
  if (c_raw(0) < -5.0 || c_raw(1) <= 0.0 || c_raw(2) <= 0.0)
  {
    return false;
  }

  // 物理限幅：防止在偶发欠激励或高噪声工况下参数发散导致控制失控
  c_raw(0) = std::min(std::max(c_raw(0), 0.0), 100.0);
  c_raw(1) = std::min(std::max(c_raw(1), 5.0), 60.0);
  c_raw(2) = std::min(std::max(c_raw(2), 0.05), 10.0);

  // c[3..11]: 对应各项阻尼、焦耳热与摩擦损耗系数，物理上必须非负
  for (int i = 3; i < 12; ++i)
  {
    c_raw(i) = std::max(c_raw(i), 0.0);
  }

  c_vec = c_raw;
  return true;
}