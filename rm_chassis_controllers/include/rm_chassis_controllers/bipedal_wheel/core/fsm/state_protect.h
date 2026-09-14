#pragma once

#include "bipedal_wheel/core/fsm/fsm_context.h"
#include <memory>

namespace bipedal_wheel_core
{

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
class Protect
{
public:
  Protect();

  void execute(FsmContext<PidType, LoggerType, RampFilterType, MovingAverageFilterType>& ctx);

private:
  double theta_des_l = 0.0, theta_des_r = 0.0, length_des_l = 0.12, length_des_r = 0.12;
  std::shared_ptr<RampFilterType> ramp_length_des_l_;
  std::shared_ptr<RampFilterType> ramp_length_des_r_;
  std::shared_ptr<RampFilterType> ramp_angle_des_l_;
  std::shared_ptr<RampFilterType> ramp_angle_des_r_;
};

}  // namespace bipedal_wheel_core
