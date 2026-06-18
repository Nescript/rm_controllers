//
// Created by Nesc on 26-6-16
//

#pragma once
#include "bipedal_wheel/core/definitions.h"
namespace bipedal_wheel_core
{
class BipedalWheelCore
{
public:
  BipedalWheelCore() = default;

  // 这一步应该引入所有的参数，包括状态机的参数，还需要初始化一些依赖注入
  bool init(bipedal_wheel_core::ControllerConfig& config);

  // 这一步应该引入硬件的状态
  void update(bipedal_wheel_core::HardwareState& state);
};
}  // namespace bipedal_wheel_core