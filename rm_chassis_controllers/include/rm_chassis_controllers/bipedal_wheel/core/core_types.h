//
// Created by Nesc on 2026-06-21.
//

#pragma once
#include <Eigen/Dense>

namespace bipedal_wheel_core
{

// 机器人状态机枚举
enum class RobotMode
{
  HANGING = 0,  // 挂起/悬空状态
  FALLEN,       // 倾覆/倒地状态
  GETTING_UP,   // 试图起立中
  STAND         // 平衡站立控制状态
};

// 左右腿枚举
enum LegIndex
{
  LEFT = 0,
  RIGHT = 1
};

constexpr int STATE_DIM = 6;
constexpr int CONTROL_DIM = 2;

// LQR state indices
enum LqrStateIndex
{
  THETA = 0,
  D_THETA = 1,
  POS = 2,
  VEL = 3,
  PITCH = 4,
  D_PITCH = 5
};

// Control input indices
enum ControlInputIndex
{
  WHEEL_T = 0,
  LEG_Tp = 1
};

// Leg orientation enum (used in stand_up)
enum class LegOrientation
{
  UNDER,
  FRONT,
  BEHIND
};

// Jump phase enum
enum class JumpPhase
{
  LEG_RETRACTION,
  JUMP_UP,
  OFF_GROUND,
  IDLE
};

// 物理与算法配置参数（由 ROS 包装层读取后传入）
struct LqrModelParams
{
  double L_weight = 0.0;   // Length weight to wheel axis
  double Lm_weight = 0.0;  // Length weight to mass center
  double l = 0.0;          // Leg rest length
  double m_w = 0.0;        // Wheel mass
  double m_p = 0.0;        // Leg mass
  double M = 0.0;          // Body mass
  double i_w = 0.0;        // Wheel inertia
  double i_p = 0.0;        // Leg inertia
  double i_m = 0.0;        // Body inertia
  double r = 0.0;          // Wheel radius
  double g = 0.0;          // Gravity acceleration
  double f_gravity = 0.0;  // Gravity Force
  double l1 = 0.0;         // Thigh link length (m)
  double l2 = 0.0;         // Calf link length (m)
  double l5 = 0.0;         // Distance offset / virtual offset (m)
};

struct ChassisGeometryParams
{
  double chassis_height = 0.0;
  double wheel_track = 0.0;
};

struct SpringParams
{
  double s2 = 0.0;
  double s3 = 0.0;
  double alpha_s = 0.0;
  double f_spring = 0.0;
};

struct ControlParams
{
  double jump_over_time = 0.0;
};

struct BiasParams
{
  double x = 0.0;
  double theta = 0.0;
  double pitch = 0.0;
  double roll = 0.0;
  double raw_pitch = 0.0;
  double raw_theta = 0.0;
};

struct LegStateThresholdParams
{
  double under_lower = 0.0;
  double under_upper = 0.0;
  double front_lower = 0.0;
  double front_upper = 0.0;
  double behind_lower = 0.0;
  double behind_upper = 0.0;
  double upstair_des_theta = 0.0;
  double upstair_des_length = 0.0;
  double upstair_exit_theta_threshold = 0.0;
  double upstair_exit_length_threshold = 0.0;
  double unstick_threshold = 0.0;
  double arrive_time_threshold = 0.0;
};

struct ControllerParams
{
  LqrModelParams model_params{};
  ChassisGeometryParams chassis_geometry{};
  SpringParams spring{};
  ControlParams control{};
  BiasParams bias{};
  LegStateThresholdParams threshold{};
  
  double default_leg_length = 0.12;
};

struct ControllerCommands
{
  Eigen::Vector3d vel_cmd = Eigen::Vector3d::Zero();
  int base_state = 0;      // Follow, raw, fallen, etc.
  bool jump_cmd = false;
  double leg_length_cmd = 0.12;
  bool overturn = false;
  const Eigen::Matrix<double, 4, 12>* coeffs = nullptr;
};

// 核心算法内部估计的机器人绝对状态（可通过 Getter 接口暴露供 ROS 观测）
struct LQRStatus
{
  double theta = 0.0;    // 虚拟腿绝对摆角（相对世界坐标系）
  double d_theta = 0.0;  // 虚拟腿绝对摆角速度
  double x = 0.0;        // 绝对水平位移
  double dx = 0.0;       // 绝对水平速度
  double pitch = 0.0;    // 底盘绝对俯仰角
  double d_pitch = 0.0;  // 底盘绝对俯仰角速度
};

// 纯粹的硬件执行器状态（输入）
struct JointState
{
  double position = 0.0;
  double vel = 0.0;
  double effort = 0.0;
};

struct LegState
{
  JointState hip;
  JointState knee;
  JointState wheel;
};

// 纯粹的底盘惯导及几何状态（输入）
struct ChassisSensorState
{
  Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero();  // 转换到 base_link 坐标系下的角速度
  Eigen::Vector3d linear_acc = Eigen::Vector3d::Zero();   // 转换到 base_link 坐标系下的线加速度
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

// 最终汇总的传感器输入
// todo 思考这里变量的名称是否合理
struct SensorMeasurements
{
  LegState leg_state[2]{};
  ChassisSensorState chassis_state{};
};

// 纯粹的关节控制器目标（输出指令）
struct JointCommand
{
  double position = 0.0;
  double velocity = 0.0;
  double effort = 0.0;  // 目标关节力矩
  double kp = 0.0;      // 刚度增益
  double kd = 0.0;      // 阻尼增益
};

struct LegCommand
{
  JointCommand hip;
  JointCommand knee;
  JointCommand wheel;
};

// 实时安全的调试数据包
struct DebugData
{
  double virtual_force = 0.0;
  double virtual_torque = 0.0;
  double lqr_error[6]{};  // 存储状态偏差量以供绘制曲线
  RobotMode mode = RobotMode::HANGING;
};

// 算法更新的最终输出包
struct ControlOutput
{
  LegCommand leg_cmd[2]{};
  // DebugData debug_data{};
};

}  // namespace bipedal_wheel_core
