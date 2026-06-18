//
// Created by Nesc on 26-8-30.
//

#pragma once
namespace bipedal_wheel_core
{
struct ControllerConfig
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
};

}  // namespace bipedal_wheel_core
