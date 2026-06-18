//
// Created by Nesc on 26-6-16
//

#include "rm_chassis_controllers/bipedal_wheel/controllers.h"
#include "ros/node_handle.h"

namespace rm_chassis_controllers
{
bool BipedalController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh)
{
  ChassisBase::init(robot_hw, root_nh, controller_nh);

  if (!initParams(controller_nh))
    return false;

  initRosInterface(root_nh, controller_nh);

  if (!initHardwareHandles(robot_hw))
  {
    return false;
  }

  if (initCoreAlgorithm())
  {
    return false;
  }
  return true;
}

/**
 * @brief Init hardware interface and extract imu, joint handle from robot_hw ptr
 *
 * @param robot_hw robot hardware ptr
 * @return true successfully get all hardware handle
 * @return false failed to get hardware interface or didint find handle
 */
bool BipedalController::initHardwareHandles(hardware_interface::RobotHW* robot_hw)
{
  auto* imu_interface = robot_hw->get<hardware_interface::ImuSensorInterface>();
  if (imu_interface == nullptr)
  {
    ROS_ERROR("[BipedalController] Failed to get ImuSensorInterface.");
    return false;
  }
  try
  {
    imu_handle_ = imu_interface->getHandle("base_imu");
  }
  catch (const hardware_interface::HardwareInterfaceException& ex)
  {
    ROS_ERROR_STREAM("[BipedalController] Failed to get imu handle: " << ex.what());
    return false;
  }

  //  gimbal_imu_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("gimbal_imu");

  auto* joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (joint_interface == nullptr)
  {
    ROS_ERROR("[BipedalController] Failed to get EffortJointInterface.");
    return false;
  }

  const std::pair<const char*, hardware_interface::JointHandle*> table[] = {
    { "left_hip_joint", &left_hip_joint_handle_ },     { "left_knee_joint", &left_knee_joint_handle_ },
    { "right_hip_joint", &right_hip_joint_handle_ },   { "right_knee_joint", &right_knee_joint_handle_ },
    { "left_wheel_joint", &left_wheel_joint_handle_ }, { "right_wheel_joint", &right_wheel_joint_handle_ }
  };
  for (const auto& t : table)
  {
    try
    {
      *t.second = joint_interface->getHandle(t.first);
      joint_handles_.push_back(t.second);
    }
    catch (const hardware_interface::HardwareInterfaceException& ex)
    {
      ROS_ERROR_STREAM("[BipedalController] Failed to get joint handle for " << t.first << ": " << ex.what());
      return false;
    }
  }

  return true;
}

bool BipedalController::initRosInterface(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  return true;
}

bool BipedalController::initParams(ros::NodeHandle& controller_nh)
{
  return true;
}

bool BipedalController::setupModelParams(ros::NodeHandle& controller_nh)
{
  return true;
}

bool BipedalController::setupChassisGeometryParams(ros::NodeHandle& controller_nh)
{
  return true;
}

bool BipedalController::setupLQR(ros::NodeHandle& controller_nh)
{
  return true;
}

bool BipedalController::setupBiasParams(ros::NodeHandle& controller_nh)
{
  return true;
}

// [will unused]
bool BipedalController::setupControlParams(ros::NodeHandle& controller_nh)
{
  return true;
}

bool BipedalController::setupThresholdParams(ros::NodeHandle& controller_nh)
{
  return true;
}

bool BipedalController::setupSpringParams(ros::NodeHandle& controller_nh)
{
  return true;
}

}  // namespace rm_chassis_controllers