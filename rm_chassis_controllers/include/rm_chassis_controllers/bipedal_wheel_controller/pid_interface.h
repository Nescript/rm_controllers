#pragma once

namespace rm_chassis_controllers
{
class PidInterface
{
public:
  virtual ~PidInterface() = default;
  virtual double computeCommand(double error, double period) = 0;
};

struct PidSet
{
  PidInterface *yaw_vel{ nullptr }, *left_leg{ nullptr }, *right_leg{ nullptr }, *theta_diff{ nullptr },
      *roll{ nullptr };
  PidInterface *left_leg_stand_up{ nullptr }, *right_leg_stand_up{ nullptr };
  PidInterface *left_leg_theta{ nullptr }, *right_leg_theta{ nullptr }, *left_leg_theta_vel{ nullptr },
      *right_leg_theta_vel{ nullptr };
  PidInterface *left_wheel_vel{ nullptr }, *right_wheel_vel{ nullptr }, *wheel_vel_diff{ nullptr };
};
}  // namespace rm_chassis_controllers
