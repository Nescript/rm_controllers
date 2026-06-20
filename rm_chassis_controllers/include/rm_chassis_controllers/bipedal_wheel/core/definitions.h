//
// Created by Nesc on 26-8-30.
//

#pragma once
#include <urdf_model/joint.h>
#include <vector>
namespace bipedal_wheel_core
{
struct ControllerParams
{
  struct ModelParams
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
  ModelParams model_params{};
};
struct HardwareState
{
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

  enum
  {
    LEFT = 0,
    RIGHT,
  };
};

}  // namespace bipedal_wheel_core
