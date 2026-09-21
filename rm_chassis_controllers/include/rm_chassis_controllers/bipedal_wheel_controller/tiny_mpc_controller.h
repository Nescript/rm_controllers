#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <ros/node_handle.h>

#include <memory>

#include <tinympc/tiny_api.hpp>

#include "bipedal_wheel_controller/definitions.h"

namespace rm_chassis_controllers
{
class TinyMpcController
{
public:
  TinyMpcController() = default;
  ~TinyMpcController();

  TinyMpcController(const TinyMpcController&) = delete;
  TinyMpcController& operator=(const TinyMpcController&) = delete;

  bool init(const ros::NodeHandle& node_handle, const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& continuous_a,
            const Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>& continuous_b,
            const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& q,
            const Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>& r);

  bool solve(const Eigen::Matrix<double, STATE_DIM, 1>& state,
             const Eigen::Matrix<double, STATE_DIM, 1>& reference,
             Eigen::Matrix<double, CONTROL_DIM, 1>& control);

  // Overwrite the horizon input bounds before solving so joint-space torque
  // limits (mapped through the leg Jacobian) can be enforced every cycle.
  bool updateInputBounds(const Eigen::Matrix<double, CONTROL_DIM, 1>& u_min,
                         const Eigen::Matrix<double, CONTROL_DIM, 1>& u_max);

  bool enabled() const { return enabled_; }

private:
  TinySolver* solver_{ nullptr };
  bool enabled_{ false };
};
}  // namespace rm_chassis_controllers