//
// 在线功率模型辨识（递归最小二乘 RLS）
// 每周期输入 v, omega, a, alpha 与功率测量值，递推累加 G = Σλ^k φφᵀ 与 b = Σλ^k φP，
// 求解正规方程 Gc = b 得到 12 维功率系数向量 c。
//
#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <limits>

class PowerModel
{
public:
  struct FitMetrics
  {
    uint64_t sample_count = 0;
    bool model_valid = false, metrics_valid = false, r2_valid = false;
    double rmse = std::numeric_limits<double>::quiet_NaN();
    double r2 = std::numeric_limits<double>::quiet_NaN();
    double bias = std::numeric_limits<double>::quiet_NaN();
  };
  FitMetrics evaluate() const;
  // Returns NaN until a valid model exists or when inputs/prediction are non-finite.
  double predict(double v, double omega, double a, double alpha) const;
  double getLambda() const { return lambda_; }
  double getAlphaRidge() const { return alpha_ridge_; }
  bool isModelValid() const { return model_valid_; }
  PowerModel();

  // 初始化：清零累计量与系数，设置遗忘因子 lambda 与岭正则 alpha_ridge
  void init(double lambda, double alpha_ridge);

  void setLambda(double lambda);
  void setAlphaRidge(double alpha_ridge);

  const Eigen::Matrix<double, 12, 1>& getC() const;
  bool setC(const Eigen::Matrix<double, 12, 1>& coefficients);
  void invalidateModel() { model_valid_ = false; }
  bool hasSufficientExcitation(uint64_t minimum_samples, double minimum_normalized_eigenvalue) const;

  // v 速度, omega 角速度, a 加速度, alpha 角加速度, P_meas 功率测量值
  bool updateD(int t, double v, double omega, double a, double alpha, double P_meas);

  // 持续激励（PE）判定：判断当前工况是否具备足够的动态激励
  bool isExcited(double v, double omega, double a, double alpha) const;

  // 样本数足够时求解正规方程并做物理约束截断，成功更新 c_vec 返回 true；失败保留旧值返回 false
  bool solve(uint64_t minimum_samples = 13);

private:
  static Eigen::Matrix<double, 12, 1> extractFeatures(double v, double omega, double a, double alpha);

  Eigen::Matrix<double, 12, 1> c_vec;    // 辨识出的功率系数
  Eigen::Matrix<double, 12, 12> A_acc;   // G = Σ λ^k φ φᵀ（充分统计量）
  Eigen::Matrix<double, 12, 1> B_acc;    // b = Σ λ^k φ P
  double lambda_;                        // 遗忘因子，有效窗口 ≈ 1/(1-λ)
  double alpha_ridge_;                   // 岭正则系数，防止激励不足时矩阵奇异
  uint64_t sample_count_;
  bool model_valid_ = false;
  Eigen::Matrix<double, 12, 12> global_g_;
  Eigen::Matrix<double, 12, 1> global_b_, feature_sum_;
  double power_square_sum_ = 0, power_sum_ = 0, power_mean_ = 0, power_m2_ = 0;
};
