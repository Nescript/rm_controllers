//
// Created by Nesc on 26-8-30.
//

#pragma once
#include <urdf_model/joint.h>
#include <Eigen/Dense>

namespace bipedal_wheel_core
{

struct LqrModelParams
{
  double L_weight;   // Length weight to wheel axis
  double Lm_weight;  // Length weight to mass center
  double l;          // Leg rest length
  double m_w;        // Wheel mass
  double m_p;        // Leg mass
  double M;          // Body mass
  double i_w;        // Wheel inertia
  double i_p;        // Leg inertia
  double i_m;        // Body inertia
  double r;          // Wheel radius
  double g;          // Gravity acceleration
  double f_gravity;  // Gravity Force
};

struct ControllerParams
{
  LqrModelParams model_params{};
};

struct LQRStatus
{
  double theta;    // 虚拟腿绝对摆角（相对世界）
  double d_theta;  // 虚拟腿绝对摆角速度
  double x;        // 绝对水平位移
  double dx;       // 绝对水平速度
  double pitch;    // 底盘绝对俯仰角
  double d_pitch;  // 底盘绝对俯仰角速度
};

struct JointStatus
{
  double vel;
  double position;
  double effort;
};

struct LegStatus
{
  JointStatus hip;
  JointStatus knee;
  JointStatus wheel;
};

struct ChassisStatus
{
  Eigen::Vector3d angular_vel;
  Eigen::Vector3d linear_acc;
  double x_vel = 0.0;  // 这个要经过卡尔曼滤波，思考如何实现
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  bool overturn = false;
};

enum
{
  LEFT = 0,
  RIGHT,
};

struct HardwareState
{
  LQRStatus lqr_status{};
  LegStatus leg_status[2]{};
  ChassisStatus chassis_status{};
};

struct LegCommand
{
  double force;     // Thrust
  double torque;    // Torque
  double input[2];  // input
};

}  // namespace bipedal_wheel_core
