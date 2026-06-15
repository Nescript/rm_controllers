//
// Created by guanlin on 25-9-3.
//

#include "bipedal_wheel_controller/controller_mode/sit_down.h"
#include "bipedal_wheel_controller/math_helpers.h"

namespace rm_chassis_controllers
{
SitDown::SitDown(PidInterface* left_wheel, PidInterface* right_wheel)
  : pid_left_wheel_(left_wheel), pid_right_wheel_(right_wheel)
{
}

LogicOutput SitDown::execute(const LogicInput& input)
{
  LogicOutput output;
  if (!input.state_changed)
  {
    output.state_changed_requested = true;
  }

  const auto& chassis_state = input.chassis_state;
  output.leg_cmd[LEFT] = { 0, 0, { 0., 0. } };
  output.leg_cmd[RIGHT] = { 0, 0, { 0., 0. } };
  output.wheel_cmd[LEFT] = 0.0;
  output.wheel_cmd[RIGHT] = 0.0;

  // Exit
  if (std::abs(chassis_state.angular_vel.y()) < 0.1 && input.base_state != 4) // Assume 4 is FALLEN
  {
    output.state_changed_requested = false;
    if (input.overturn)
    {
      output.next_mode = BalanceMode::RECOVER;
    }
    else
    {
      output.next_mode = BalanceMode::STAND_UP;
    }
  }
  
  return output;
}
}  // namespace rm_chassis_controllers
