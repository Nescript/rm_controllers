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
  ChassisBase<rm_control::RobotStateInterface, hardware_interface::ImuSensorInterface,
              hardware_interface::EffortJointInterface>::init(robot_hw, root_nh, controller_nh);

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

/********************************** 应该移动到 .h 文件 ***********************************/
inline void quatToRPY(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw)
{
  double as = std::min(-2. * (q.x * q.z - q.w * q.y), .99999);
  yaw = std::atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
  pitch = std::asin(as);
  roll = std::atan2(2 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
}

inline void setJointCommands(std::vector<hardware_interface::JointHandle*>& joints,
                             const bipedal_wheel_core::ControlOutput& ctrl_out)
{
  if (joints.size() != 6)
    throw std::runtime_error("Joint handle vector size must be 6!");

  // 关节顺序在 initHardwareHandles 中初始化为:
  // left_hip, left_knee, right_hip, right_knee, left_wheel, right_wheel
  joints[0]->setCommand(ctrl_out.leg_cmd[bipedal_wheel_core::LEFT].hip.effort);
  joints[1]->setCommand(ctrl_out.leg_cmd[bipedal_wheel_core::LEFT].knee.effort);
  joints[2]->setCommand(ctrl_out.leg_cmd[bipedal_wheel_core::RIGHT].hip.effort);
  joints[3]->setCommand(ctrl_out.leg_cmd[bipedal_wheel_core::RIGHT].knee.effort);
  joints[4]->setCommand(ctrl_out.leg_cmd[bipedal_wheel_core::LEFT].wheel.effort);
  joints[5]->setCommand(ctrl_out.leg_cmd[bipedal_wheel_core::RIGHT].wheel.effort);
}

void BipedalController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  // get info from imu
  geometry_msgs::Vector3 gyro, acc;
  /************************************************************** */
  gyro.x = imu_handle_.getAngularVelocity()[0];
  gyro.y = imu_handle_.getAngularVelocity()[1];
  gyro.z = imu_handle_.getAngularVelocity()[2];
  acc.x = imu_handle_.getLinearAcceleration()[0];
  acc.y = imu_handle_.getLinearAcceleration()[1];
  acc.z = imu_handle_.getLinearAcceleration()[2];
  tf2::Transform odom2imu, imu2base, odom2base;
  geometry_msgs::Vector3 angular_vel_base{}, linear_acc_base{};
  double roll{}, pitch{}, yaw{};

  bipedal_wheel_core::SensorMeasurements sens_in{};

  try
  {
    // 将 imu 数据变换到 base link 坐标系
    tf2::doTransform(gyro, angular_vel_base,
                     robot_state_handle_.lookupTransform("base_link", imu_handle_.getFrameId(), time));
    // get imu to base transform
    geometry_msgs::TransformStamped tf_msg;
    tf_msg = robot_state_handle_.lookupTransform(imu_handle_.getFrameId(), "base_link", time);
    tf2::fromMsg(tf_msg.transform, imu2base);
    // get odom to imu transform
    tf2::Quaternion odom2imu_quaternion;
    tf2::Vector3 odom2imu_origin;
    odom2imu_quaternion.setValue(imu_handle_.getOrientation()[0], imu_handle_.getOrientation()[1],
                                 imu_handle_.getOrientation()[2], imu_handle_.getOrientation()[3]);
    odom2imu_origin.setValue(0, 0, 0);
    odom2imu.setOrigin(odom2imu_origin);
    odom2imu.setRotation(odom2imu_quaternion);
    odom2base = odom2imu * imu2base;
    quatToRPY(toMsg(odom2base).rotation, roll, pitch, yaw);

    tf_msg.transform = tf2::toMsg(odom2imu.inverse());
    tf_msg.header.stamp = time;
    tf2::doTransform(acc, linear_acc_base, tf_msg);

    // 打包底盘传感器数据
    sens_in.chassis_state.angular_vel << angular_vel_base.x, angular_vel_base.y, angular_vel_base.z;
    sens_in.chassis_state.linear_acc << linear_acc_base.x, linear_acc_base.y, linear_acc_base.z;
    sens_in.chassis_state.roll = roll;
    sens_in.chassis_state.pitch = pitch;
    sens_in.chassis_state.yaw = yaw;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_ONCE("%s", ex.what());
    bipedal_wheel_core::ControlOutput zero_cmd{};
    setJointCommands(joint_handles_, zero_cmd);
    return;
  }

  // 1. 打包左腿关节状态 (hip, knee, wheel)
  sens_in.leg_state[bipedal_wheel_core::LEFT].hip.position = left_hip_joint_handle_.getPosition();
  sens_in.leg_state[bipedal_wheel_core::LEFT].hip.vel = left_hip_joint_handle_.getVelocity();
  sens_in.leg_state[bipedal_wheel_core::LEFT].hip.effort = left_hip_joint_handle_.getEffort();

  sens_in.leg_state[bipedal_wheel_core::LEFT].knee.position = left_knee_joint_handle_.getPosition();
  sens_in.leg_state[bipedal_wheel_core::LEFT].knee.vel = left_knee_joint_handle_.getVelocity();
  sens_in.leg_state[bipedal_wheel_core::LEFT].knee.effort = left_knee_joint_handle_.getEffort();

  sens_in.leg_state[bipedal_wheel_core::LEFT].wheel.position = left_wheel_joint_handle_.getPosition();
  sens_in.leg_state[bipedal_wheel_core::LEFT].wheel.vel = left_wheel_joint_handle_.getVelocity();
  sens_in.leg_state[bipedal_wheel_core::LEFT].wheel.effort = left_wheel_joint_handle_.getEffort();

  // 2. 打包右腿关节状态 (hip, knee, wheel)
  sens_in.leg_state[bipedal_wheel_core::RIGHT].hip.position = right_hip_joint_handle_.getPosition();
  sens_in.leg_state[bipedal_wheel_core::RIGHT].hip.vel = right_hip_joint_handle_.getVelocity();
  sens_in.leg_state[bipedal_wheel_core::RIGHT].hip.effort = right_hip_joint_handle_.getEffort();

  sens_in.leg_state[bipedal_wheel_core::RIGHT].knee.position = right_knee_joint_handle_.getPosition();
  sens_in.leg_state[bipedal_wheel_core::RIGHT].knee.vel = right_knee_joint_handle_.getVelocity();
  sens_in.leg_state[bipedal_wheel_core::RIGHT].knee.effort = right_knee_joint_handle_.getEffort();

  sens_in.leg_state[bipedal_wheel_core::RIGHT].wheel.position = right_wheel_joint_handle_.getPosition();
  sens_in.leg_state[bipedal_wheel_core::RIGHT].wheel.vel = right_wheel_joint_handle_.getVelocity();
  sens_in.leg_state[bipedal_wheel_core::RIGHT].wheel.effort = right_wheel_joint_handle_.getEffort();

  // 3. 传入时间步长 dt 与传感器测量，运行核心算法
  double dt = period.toSec();
  bipedal_wheel_core::ControlOutput ctrl_out = core_.update(dt, sens_in);

  // 4. 将输出控制量应用给电机
  setJointCommands(joint_handles_, ctrl_out);
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