//
// Created by Nesc on 26-6-16
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include "rm_chassis_controllers/chassis_base.h"

namespace rm_chassis_controllers
{

class BipedalController : public ChassisBase<rm_control::RobotStateInterface, hardware_interface::>

}