#pragma once

#include <Eigen/Dense>
#include <memory>
#include "bipedal_wheel_controller/definitions.h"

namespace rm_chassis_controllers
{
struct UserInstruction
{
  double x_vel_cmd{ 0.0 };
  double yaw_vel_cmd{ 0.0 };
  double leg_length_cmd{ 0.0 };
  bool jump_cmd{ false };
};

struct LogicInput
{
  double time;
  double period;
  ChassisState chassis_state;
  LegState leg_state[2];
  UserInstruction instruction;

  std::shared_ptr<ModelParams> model_params;
  std::shared_ptr<ControlParams> control_params;
  std::shared_ptr<BiasParams> bias_params;
  std::shared_ptr<LegStateThresholdParams> leg_threshold_params;
  std::shared_ptr<ChassisGeometryParams> chassis_geometry_params;
  std::shared_ptr<SpringParams> spring_params;

  double left_wheel_vel;
  double right_wheel_vel;

  int base_state;
  bool overturn;
  bool state_changed;
  bool complete_stand;
  bool move_flag;
  double default_leg_length;
  Eigen::Matrix<double, 4, 12> coeffs;
};

struct LogicOutput
{
  LegCommand leg_cmd[2];
  double wheel_cmd[2];
  int next_mode{ -1 };  // -1 means no change
  bool state_changed_requested{ false };
  bool complete_stand_requested{ false };
  bool jump_cmd_requested{ false };
  bool move_flag_requested{ false };
  bool clear_status_requested{ false };
  bool recovery_leg_spd_turnback_requested{ false };
};

}  // namespace rm_chassis_controllers
