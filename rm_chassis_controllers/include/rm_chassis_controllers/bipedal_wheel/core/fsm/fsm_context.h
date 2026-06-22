#pragma once

#include "bipedal_wheel/core/core_types.h"
#include "bipedal_wheel/core/vmc/VMC.h"
#include <vector>

namespace bipedal_wheel_core
{

template <
    typename PidType,
    typename LoggerType,
    typename RampFilterType,
    typename MovingAverageFilterType
>
struct FsmContext
{
  const ControllerParams& config;
  const SensorMeasurements& sens_in;
  const ControllerCommands& cmd_in;

  ControlOutput& control_output;
  LQRStatus& lqr_status;
  RobotMode& current_mode;

  // Shared state indicators
  bool& complete_stand;
  bool& overturn;
  bool& balance_state_changed;
  bool& recovery_leg_spd_turnback;

  // VMC Kinematic Solvers
  VMC* left_vmc = nullptr;
  VMC* right_vmc = nullptr;

  // Timing
  double dt = 0.001;

  // Injected helper references
  LoggerType& logger;
  
  // PID and Filter parameters mapping/references
  const std::vector<PidType*>& pid_legs;
  const std::vector<PidType*>& pid_thetas;
  const std::vector<PidType*>& pid_wheels;
  PidType* pid_yaw_vel = nullptr;
  PidType* pid_theta_diff = nullptr;
  PidType* pid_roll = nullptr;
  PidType* pid_wheel_vel_diff = nullptr;
};

}  // namespace bipedal_wheel_core
