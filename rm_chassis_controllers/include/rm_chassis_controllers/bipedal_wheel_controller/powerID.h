//
// 在线功率模型辨识（递归最小二乘 RLS）
// 每周期输入 v, omega, a, alpha 与功率测量值，递推累加 G = Σλ^k φφᵀ 与 b = Σλ^k φP，
// 求解正规方程 Gc = b 得到 12 维功率系数向量 c。
//
#pragma once

#include <Eigen/Dense>

class PowerModel
{
public:
  PowerModel();

  // 初始化：清零累计量与系数，设置遗忘因子 lambda 与岭正则 alpha_ridge
  void init(double lambda, double alpha_ridge);

  void setLambda(double lambda);
  void setAlphaRidge(double alpha_ridge);

  const Eigen::Matrix<double, 12, 1>& getC() const;

  // v 速度, omega 角速度, a 加速度, alpha 角加速度, P_meas 功率测量值
  void updateD(int t, double v, double omega, double a, double alpha, double P_meas);

  // 持续激励（PE）判定：判断当前工况是否具备足够的动态激励
  bool isExcited(double v, double omega, double a, double alpha) const;

  // 样本数足够时求解正规方程并做物理约束截断，成功更新 c_vec 返回 true；失败保留旧值返回 false
  bool solve();

private:
  static Eigen::Matrix<double, 12, 1> extractFeatures(double v, double omega, double a, double alpha);

  Eigen::Matrix<double, 12, 1> c_vec;    // 辨识出的功率系数
  Eigen::Matrix<double, 12, 12> A_acc;   // G = Σ λ^k φ φᵀ（充分统计量）
  Eigen::Matrix<double, 12, 1> B_acc;    // b = Σ λ^k φ P
  double lambda_;                        // 遗忘因子，有效窗口 ≈ 1/(1-λ)
  double alpha_ridge_;                   // 岭正则系数，防止激励不足时矩阵奇异
  int sample_count_;
};
