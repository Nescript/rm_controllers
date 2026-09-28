#include "bipedal_wheel/core/vmc/VMC.h"
#include "bipedal_wheel/core/fsm/jump_contact_detector.h"
#include "bipedal_wheel/core/fsm/jump_control_policy.h"
#include <gtest/gtest.h>
#include <array>
#include <cmath>

namespace
{
using bipedal_wheel_core::VMC;
constexpr double kL1 = 0.21;
constexpr double kL2 = 0.248;

VMC makeFiveBar()
{
  return VMC(kL1, kL2, kL2, kL1, 0.0, true);
}

void checkJacobian(VMC vmc, double phi1, double phi4)
{
  constexpr double step = 1e-6;
  vmc.calc_jacobian(phi1, phi4);
  double radial[2], angular[2];
  vmc.leg_conv(1.0, 0.0, radial);
  vmc.leg_conv(0.0, 1.0, angular);
  for (int col = 0; col < 2; ++col)
  {
    vmc.leg_pos(phi1 + (col == 0 ? step : 0.0), phi4 + (col == 1 ? step : 0.0));
    const auto plus = vmc.getPos();
    vmc.leg_pos(phi1 - (col == 0 ? step : 0.0), phi4 - (col == 1 ? step : 0.0));
    const auto minus = vmc.getPos();
    EXPECT_NEAR(radial[col], (plus.L0 - minus.L0) / (2.0 * step), 1e-7);
    const double angle_delta = std::atan2(std::sin(plus.theta - minus.theta),
                                          std::cos(plus.theta - minus.theta));
    EXPECT_NEAR(angular[col], angle_delta / (2.0 * step), 1e-7);
  }
}

TEST(BipedalWheelVmc, RealFiveBarZeroOffsets)
{
  auto vmc = makeFiveBar();
  // At raw motor zero, phi1 = pi and phi4 = 0: the wheel is below the common pivot.
  vmc.leg_pos(M_PI, 0.0);
  EXPECT_NEAR(vmc.getPos().L0, std::sqrt(kL2 * kL2 - kL1 * kL1), 1e-12);
  EXPECT_NEAR(vmc.getPos().theta, 0.0, 1e-12);
}

TEST(BipedalWheelVmc, FiveBarJacobianMatchesPositionDerivative)
{
  for (double hip : { -0.8, -0.4, 0.0, 0.3, 0.6 })
    for (double knee : { -0.3, 0.0, 0.4, 0.8 })
      checkJacobian(makeFiveBar(), M_PI + hip, knee);
  // Nonzero pivot spacing and asymmetric rods remain supported.
  checkJacobian(VMC(0.21, 0.248, 0.26, 0.20, 0.04, true), 2.7, 0.4);
}

TEST(BipedalWheelVmc, VirtualForceRoundTripAndPower)
{
  for (bool five_bar : { false, true })
  {
    VMC vmc(kL1, kL2, kL2, kL1, 0.0, five_bar);
    vmc.calc_jacobian(2.7, 0.4);
    for (const auto& force : { std::array<double, 2>{ 110.0, 0.0 }, { 0.0, 4.0 }, { -75.0, 3.5 } })
    {
      double torques[2];
      vmc.leg_conv(force[0], force[1], torques);
      vmc.leg_conv_t(torques[0], torques[1]);
      EXPECT_NEAR(vmc.getForceReal().F, force[0], 1e-10);
      EXPECT_NEAR(vmc.getForceReal().Tp, force[1], 1e-10);
      vmc.leg_spd(0.3, -0.7);
      EXPECT_NEAR(torques[0] * 0.3 + torques[1] * -0.7,
                  force[0] * vmc.getSpd().dL0 + force[1] * vmc.getSpd().dTheta, 1e-10);
    }
  }
}

TEST(BipedalWheelVmc, JointTorqueBoundsMatchConvertedMotorCommands)
{
  for (bool five_bar : { false, true })
  {
    VMC vmc(kL1, kL2, kL2, kL1, 0.0, five_bar);
    vmc.calc_jacobian(2.7, 0.4);
    constexpr double force = 50.0;
    constexpr double pitch_correction = 2.0;
    double lower, upper;
    ASSERT_TRUE(vmc.leg_torque_bounds(force, pitch_correction, -40.0, 40.0, lower, upper));
    ASSERT_LT(lower, upper);
    for (double pitch_torque : { lower, (lower + upper) / 2.0, upper })
    {
      double motor_torques[2];
      vmc.leg_conv(force, pitch_torque + pitch_correction, motor_torques);
      for (double torque : motor_torques)
      {
        EXPECT_GE(torque, -40.0 - 1e-9);
        EXPECT_LE(torque, 40.0 + 1e-9);
      }
    }

    double motor_torques[2];
    vmc.leg_conv(force, lower - 1.0 + pitch_correction, motor_torques);
    EXPECT_TRUE(motor_torques[0] < -40.0 || motor_torques[0] > 40.0 || motor_torques[1] < -40.0 ||
                motor_torques[1] > 40.0);
    vmc.leg_conv(force, upper + 1.0 + pitch_correction, motor_torques);
    EXPECT_TRUE(motor_torques[0] < -40.0 || motor_torques[0] > 40.0 || motor_torques[1] < -40.0 ||
                motor_torques[1] > 40.0);
    EXPECT_FALSE(vmc.leg_torque_bounds(1e6, pitch_correction, -40.0, 40.0, lower, upper));
  }
}

TEST(BipedalWheelVmc, VelocityFiltersAreIndependentAndResettable)
{
  auto left = makeFiveBar();
  auto right = makeFiveBar();
  left.calc_jacobian(2.7, 0.4);
  right.calc_jacobian(2.7, 0.4);
  left.leg_spd(2.0, -1.0);
  const auto initial = left.getSpd();
  right.leg_spd(0.0, 0.0);
  EXPECT_DOUBLE_EQ(right.getSpd().dL0, 0.0);
  EXPECT_DOUBLE_EQ(right.getSpd().dTheta, 0.0);
  left.leg_spd(0.0, 0.0);
  EXPECT_NEAR(left.getSpd().dL0, 0.6 * initial.dL0, 1e-12);
  EXPECT_NEAR(left.getSpd().dTheta, 0.6 * initial.dTheta, 1e-12);
  right.leg_spd(0.0, 0.0);
  EXPECT_DOUBLE_EQ(right.getSpd().dL0, 0.0);
  EXPECT_DOUBLE_EQ(right.getSpd().dTheta, 0.0);
  left = makeFiveBar();
  left.calc_jacobian(2.7, 0.4);
  left.leg_spd(0.0, 0.0);
  EXPECT_DOUBLE_EQ(left.getSpd().dL0, 0.0);
  EXPECT_DOUBLE_EQ(left.getSpd().dTheta, 0.0);
}

TEST(BipedalWheelVmc, SingularAndUnreachableFiveBarHaveNoForceGain)
{
  for (auto vmc : { makeFiveBar(), VMC(0.21, 0.1, 0.1, 0.21, 0.0, true) })
  {
    vmc.calc_jacobian(2.7, 0.4);
    // Coincident elbows or an unreachable assembly must not retain the previous Jacobian.
    const double phi1 = vmc.getL2() == kL2 ? 0.0 : M_PI;
    vmc.calc_jacobian(phi1, 0.0);
    vmc.leg_pos(phi1, 0.0);
    EXPECT_TRUE(std::isfinite(vmc.getPos().L0));
    EXPECT_TRUE(std::isfinite(vmc.getPos().theta));
    double torques[2];
    vmc.leg_conv(110.0, 5.0, torques);
    EXPECT_DOUBLE_EQ(torques[0], 0.0);
    EXPECT_DOUBLE_EQ(torques[1], 0.0);
    vmc.leg_conv_t(10.0, -10.0);
    EXPECT_DOUBLE_EQ(vmc.getForceReal().F, 0.0);
    EXPECT_DOUBLE_EQ(vmc.getForceReal().Tp, 0.0);
  }
}

TEST(BipedalWheelVmc, SerialKinematicsRemainAvailable)
{
  VMC vmc(kL1, kL2);
  vmc.leg_pos(0.0, M_PI_2);
  EXPECT_NEAR(vmc.getPos().L0, std::hypot(kL1, kL2), 1e-12);
  EXPECT_NEAR(vmc.getPos().theta, std::atan2(kL2, kL1), 1e-12);
  checkJacobian(vmc, 0.3, -0.7);
}

TEST(BipedalWheelJump, JumpContactDetectorFlightAndLanding)
{
  bipedal_wheel_core::JumpContactDetector detector;
  const double unstick_threshold = 15.0;
  const double dt = 0.002;  // 2ms (500Hz control loop)

  // 1. Initial ground contact before airborne should NOT trigger landing
  for (int i = 0; i < 50; ++i)
  {
    EXPECT_FALSE(detector.update(50.0, 50.0, unstick_threshold, dt));
  }

  // 2. Flight starts: force drops below unstick_threshold (e.g. 5.0 N)
  // For first 4 steps (8 ms < 10 ms), airborne is not yet confirmed
  for (int i = 0; i < 4; ++i)
  {
    EXPECT_FALSE(detector.update(5.0, 5.0, unstick_threshold, dt));
  }
  // 5th step (10 ms): airborne confirmed, but still in flight
  EXPECT_FALSE(detector.update(5.0, 5.0, unstick_threshold, dt));
  EXPECT_TRUE(detector.isAirborne());

  // 3. In mid-air, a temporary force spike below (unstick_threshold + 5.0) (e.g. 18 N) does NOT trigger landing
  EXPECT_FALSE(detector.update(18.0, 5.0, unstick_threshold, dt));

  // 4. Touchdown on left leg: force >= contact_threshold (20.0 N)
  // Must hold for >= 10 ms (5 cycles at 2ms)
  for (int i = 0; i < 4; ++i)
  {
    EXPECT_FALSE(detector.update(30.0, 5.0, unstick_threshold, dt));
  }
  // On 5th cycle (10ms of contact), landing confirmed!
  EXPECT_TRUE(detector.update(30.0, 5.0, unstick_threshold, dt));

  // 5. Reset clears state: now ground force again returns false
  detector.reset();
  EXPECT_FALSE(detector.isAirborne());
  EXPECT_FALSE(detector.update(30.0, 30.0, unstick_threshold, dt));
}

TEST(BipedalWheelJump, OnlyLegAngleFeedbackInConfirmedFlight)
{
  using namespace bipedal_wheel_core;
  Eigen::Matrix<double, STATE_DIM, 1> error;
  error << 1, 2, 3, 4, 5, 6;
  const auto original = error;

  maskJumpFeedback(error, JumpPhase::IDLE, false);
  EXPECT_TRUE(error.isApprox(original));

  maskJumpFeedback(error, JumpPhase::LEG_RETRACTION, false);
  EXPECT_DOUBLE_EQ(error(POS), 0.0);
  EXPECT_DOUBLE_EQ(error(VEL), 0.0);
  EXPECT_DOUBLE_EQ(error(PITCH), 5.0);

  error = original;
  maskJumpFeedback(error, JumpPhase::JUMP_UP, false);
  EXPECT_DOUBLE_EQ(error(THETA), 1.0);
  EXPECT_DOUBLE_EQ(error(D_THETA), 2.0);
  EXPECT_DOUBLE_EQ(error(POS), 0.0);
  EXPECT_DOUBLE_EQ(error(VEL), 0.0);
  EXPECT_DOUBLE_EQ(error(PITCH), 5.0);
  EXPECT_DOUBLE_EQ(error(D_PITCH), 6.0);

  for (const auto phase : { JumpPhase::JUMP_UP, JumpPhase::OFF_GROUND })
  {
    error = original;
    maskJumpFeedback(error, phase, true);
    EXPECT_DOUBLE_EQ(error(THETA), 1.0);
    EXPECT_DOUBLE_EQ(error(D_THETA), 2.0);
    EXPECT_TRUE(error.tail<4>().isZero());
  }
}
}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
