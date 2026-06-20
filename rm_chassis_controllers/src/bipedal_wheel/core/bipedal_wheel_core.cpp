#include "bipedal_wheel/core/bipedal_wheel_core.h"
#include "bipedal_wheel/core/definitions.h"
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

void BipedalWheelCore::update(bipedal_wheel_core::HardwareState& state)
{
}
}  // namespace bipedal_wheel_core