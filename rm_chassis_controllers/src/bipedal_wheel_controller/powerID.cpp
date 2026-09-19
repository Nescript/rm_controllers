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
  model_valid_ = false;
  global_g_.setZero();
  global_b_.setZero();
  feature_sum_.setZero();
  power_square_sum_ = power_sum_ = power_mean_ = power_m2_ = 0;
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

bool PowerModel::setC(const Eigen::Matrix<double, 12, 1>& coefficients)
{
  if (!coefficients.allFinite() || coefficients.minCoeff() < 0.0)
    return false;
  c_vec = coefficients;
  model_valid_ = true;
  return true;
}

bool PowerModel::hasSufficientExcitation(uint64_t minimum_samples, double minimum_normalized_eigenvalue) const
{
  if (sample_count_ < minimum_samples || !std::isfinite(minimum_normalized_eigenvalue) ||
      minimum_normalized_eigenvalue < 0.0)
    return false;

  Eigen::Matrix<double, 12, 1> scale;
  for (int i = 0; i < scale.rows(); ++i)
  {
    if (!std::isfinite(A_acc(i, i)) || A_acc(i, i) <= std::numeric_limits<double>::epsilon())
      return false;
    scale(i) = std::sqrt(A_acc(i, i));
  }
  Eigen::Matrix<double, 12, 12> normalized;
  for (int row = 0; row < normalized.rows(); ++row)
    for (int column = 0; column < normalized.cols(); ++column)
      normalized(row, column) = A_acc(row, column) / (scale(row) * scale(column));
  normalized = 0.5 * (normalized + normalized.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 12, 12>> eigen_solver(normalized, Eigen::EigenvaluesOnly);
  return eigen_solver.info() == Eigen::Success && eigen_solver.eigenvalues().allFinite() &&
      eigen_solver.eigenvalues().minCoeff() >= minimum_normalized_eigenvalue;
}

bool PowerModel::updateD(int t, double v, double omega, double a, double alpha, double P_meas)
{
  (void)t;  // 预留：暂未使用
  Eigen::Matrix<double, 12, 1> phi_k = extractFeatures(v, omega, a, alpha);
  if (!phi_k.allFinite() || !std::isfinite(P_meas) ||
      !(phi_k * phi_k.transpose()).allFinite() || !(P_meas * phi_k).allFinite() ||
      !std::isfinite(P_meas * P_meas))
    return false;
  A_acc = lambda_ * A_acc + phi_k * phi_k.transpose();
  B_acc = lambda_ * B_acc + P_meas * phi_k;
  sample_count_++;
  global_g_ += phi_k * phi_k.transpose();
  global_b_ += P_meas * phi_k;
  feature_sum_ += phi_k;
  power_sum_ += P_meas;
  power_square_sum_ += P_meas * P_meas;
  const double delta = P_meas - power_mean_;
  power_mean_ += delta / sample_count_;
  power_m2_ += delta * (P_meas - power_mean_);
  return true;
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

bool PowerModel::solve(uint64_t minimum_samples)
{
  if (sample_count_ < minimum_samples)
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
  model_valid_ = true;
  return true;
}

PowerModel::FitMetrics PowerModel::evaluate() const
{
  FitMetrics result;
  result.sample_count = sample_count_;
  result.model_valid = model_valid_;
  if (!model_valid_ || sample_count_ <= 12)
    return result;
  const double predicted_square_sum = c_vec.dot(global_g_ * c_vec);
  const double cross = 2.0 * c_vec.dot(global_b_);
  double sse = predicted_square_sum - cross + power_square_sum_;
  const double tolerance = 128 * std::numeric_limits<double>::epsilon() *
      std::max(1.0, std::abs(predicted_square_sum) + std::abs(cross) + power_square_sum_);
  const double bias = (c_vec.dot(feature_sum_) - power_sum_) / sample_count_;
  if (!std::isfinite(sse) || !std::isfinite(bias) || sse < -tolerance)
    return result;
  sse = std::max(0.0, sse);
  result.rmse = std::sqrt(sse / sample_count_);
  result.bias = bias;
  result.metrics_valid = true;
  if (std::isfinite(power_m2_) && power_m2_ >
      128 * std::numeric_limits<double>::epsilon() * std::max(1.0, power_square_sum_))
  {
    result.r2 = 1.0 - sse / power_m2_;
    result.r2_valid = std::isfinite(result.r2);
  }
  return result;
}


double PowerModel::predict(double v, double omega, double a, double alpha) const
{
  if (!model_valid_)
    return std::numeric_limits<double>::quiet_NaN();
  const auto phi = extractFeatures(v, omega, a, alpha);
  if (!phi.allFinite())
    return std::numeric_limits<double>::quiet_NaN();
  const double power = c_vec.dot(phi);
  return std::isfinite(power) ? power : std::numeric_limits<double>::quiet_NaN();
}
