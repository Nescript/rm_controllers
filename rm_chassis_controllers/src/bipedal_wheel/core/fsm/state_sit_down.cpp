#include "bipedal_wheel/core/fsm/state_sit_down.h"
#include "bipedal_wheel/core/helper_functions.h"
#include "bipedal_wheel/controllers.h"
#include <cmath>

namespace bipedal_wheel_core
{

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
void SitDown<PidType, LoggerType, RampFilterType, MovingAverageFilterType>::execute(FsmContext<PidType, LoggerType, RampFilterType, MovingAverageFilterType>& ctx)
{
  if (!ctx.balance_state_changed)
  {
    ctx.logger.info("[balance] Enter SIT_DOWN");
    ctx.balance_state_changed = true;
  }

  // Set motor command outputs to 0 (sit down compliance)
  for (int leg = 0; leg < 2; ++leg)
  {
    ctx.control_output.leg_cmd[leg].hip.position = 0.0;
    ctx.control_output.leg_cmd[leg].hip.velocity = 0.0;
    ctx.control_output.leg_cmd[leg].hip.effort = 0.0;
    ctx.control_output.leg_cmd[leg].hip.kp = 0.0;
    ctx.control_output.leg_cmd[leg].hip.kd = 0.0;

    ctx.control_output.leg_cmd[leg].knee.position = 0.0;
    ctx.control_output.leg_cmd[leg].knee.velocity = 0.0;
    ctx.control_output.leg_cmd[leg].knee.effort = 0.0;
    ctx.control_output.leg_cmd[leg].knee.kp = 0.0;
    ctx.control_output.leg_cmd[leg].knee.kd = 0.0;

    ctx.control_output.leg_cmd[leg].wheel.position = 0.0;
    ctx.control_output.leg_cmd[leg].wheel.velocity = 0.0;
    ctx.control_output.leg_cmd[leg].wheel.effort = 0.0;
    ctx.control_output.leg_cmd[leg].wheel.kp = 0.0;
    ctx.control_output.leg_cmd[leg].wheel.kd = 0.0;
  }

  // Exit conditions: pitch angular velocity is small and controller commands show we're not fallen
  if (std::abs(ctx.sens_in.chassis_state.angular_vel.y()) < 0.1 && ctx.cmd_in.base_state != 3)
  {
    ctx.balance_state_changed = false;
    if (ctx.overturn)
    {
      ctx.current_physical_state = RobotPhysicalState::GETTING_UP;
    }
    else
    {
      ctx.current_physical_state = RobotPhysicalState::STAND;
    }
    ctx.logger.info("[balance] Exit SIT_DOWN");
  }
}

// Explicit instantiation for the ROS controller's injected types.
template class SitDown<rm_chassis_controllers::RosPidWrapper, rm_chassis_controllers::RosLoggerWrapper,
                      RampFilter<double>, MovingAverageFilter<double>>;

}  // namespace bipedal_wheel_core
