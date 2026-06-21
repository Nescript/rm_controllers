//
// Created by Nesc on 26-6-16
//

#pragma once
#include "bipedal_wheel/core/core_types.h"
namespace bipedal_wheel_core
{
class BipedalWheelCore
{
public:
  BipedalWheelCore() = default;

  // 这一步应该引入所有的参数，包括状态机的参数，还需要初始化一些依赖注入
  bool init(const bipedal_wheel_core::ControllerParams& config);

  // 传入时间步长与传感器数据，返回控制输出
  bipedal_wheel_core::ControlOutput update(double dt, const bipedal_wheel_core::SensorMeasurements& sens_in);

  // 重置卡尔曼滤波、状态估计以及控制器积分状态
  void reset();

  // 暴露只读状态供 ROS 观测
  const bipedal_wheel_core::LQRStatus& getLqrStatus() const { return lqr_status_; }
  bipedal_wheel_core::RobotMode getRobotMode() const { return robot_mode_; }

private:
  bipedal_wheel_core::ControllerParams config_;
  bipedal_wheel_core::LQRStatus lqr_status_;
  bipedal_wheel_core::RobotMode robot_mode_ = bipedal_wheel_core::RobotMode::HANGING;

  void updateEstimation(double dt, const bipedal_wheel_core::SensorMeasurements& sens_in);
};
}  // namespace bipedal_wheel_core