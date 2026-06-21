#include "bipedal_wheel/core/bipedal_wheel_core.h"
#include <cmath>
#include <iostream>

namespace bipedal_wheel_core
{

// 下面的函数可以考虑转为存储config对象的指针，但是要考虑其生命周期
bool BipedalWheelCore::init(const bipedal_wheel_core::ControllerParams& config)
{
  const auto& params = config.model_params;

  if (params.m_w <= 0.0 || params.m_p <= 0.0 || params.M <= 0.0 || params.i_w <= 0.0 || params.i_p <= 0.0 ||
      params.i_m <= 0.0 || params.r <= 0.0 || params.l <= 0.0 || params.g <= 0.0)
  {
    std::cerr << "[BipedalWheelCore] Parameter check failed: Physical parameters (mass, inertia, length, gravity) must "
                 "be greater than 0."
              << std::endl;
    return false;
  }

  if (!std::isfinite(params.L_weight) || !std::isfinite(params.Lm_weight) || !std::isfinite(params.l) ||
      !std::isfinite(params.m_w) || !std::isfinite(params.m_p) || !std::isfinite(params.M) ||
      !std::isfinite(params.i_w) || !std::isfinite(params.i_p) || !std::isfinite(params.i_m) ||
      !std::isfinite(params.r) || !std::isfinite(params.g) || !std::isfinite(params.f_gravity))
  {
    std::cerr << "[BipedalWheelCore] Parameter check failed: Detected non-finite values (NaN or Inf) in config."
              << std::endl;
    return false;
  }

  config_ = config;
  return true;
}

bipedal_wheel_core::ControlOutput BipedalWheelCore::update(double dt, const bipedal_wheel_core::SensorMeasurements& sens_in)
{
  // 1. 更新卡尔曼滤波等状态估计
  updateEstimation(dt, sens_in);

  // 2. 状态机和控制安全计算

  // 3. 计算输出关节力矩/PD控制目标
  bipedal_wheel_core::ControlOutput output{};
  return output;
}

void BipedalWheelCore::reset()
{
  // 重置内部状态估计
  lqr_status_ = bipedal_wheel_core::LQRStatus{};
  robot_mode_ = bipedal_wheel_core::RobotMode::HANGING;
}

void BipedalWheelCore::updateEstimation(double dt, const bipedal_wheel_core::SensorMeasurements& sens_in)
{
  // 状态估计器占位符，由您之后勤学精进完成具体融合与滤波算法
}

}  // namespace bipedal_wheel_core