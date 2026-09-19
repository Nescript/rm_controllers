#include <gtest/gtest.h>
#include <rm_chassis_controllers/bipedal_wheel_controller/jump_contact_detector.h>

using rm_chassis_controllers::JumpContactDetector;

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(JumpContactDetector, RequiresContinuousFlightBeforeContact)
{
  JumpContactDetector detector;
  for (int i = 0; i < 100; ++i)
    EXPECT_FALSE(detector.update(40, 40, 15, 0.002));
  EXPECT_FALSE(detector.update(0, 0, 15, 0.006));
  EXPECT_FALSE(detector.update(40, 0, 15, 0.006));
  EXPECT_FALSE(detector.update(0, 0, 15, 0.006));
  EXPECT_FALSE(detector.update(40, 40, 15, 0.02));
}

TEST(JumpContactDetector, EitherWheelEndsFlightAfterContinuousContact)
{
  for (bool left : { false, true })
  {
    JumpContactDetector detector;
    EXPECT_FALSE(detector.update(0, 0, 15, 0.01));
    EXPECT_FALSE(detector.update(left ? 25 : 0, left ? 0 : 25, 15, 0.006));
    EXPECT_TRUE(detector.update(left ? 25 : 0, left ? 0 : 25, 15, 0.006));
  }
}

TEST(JumpContactDetector, RejectsSpikesChatterAndAlternatingWheels)
{
  JumpContactDetector detector;
  EXPECT_FALSE(detector.update(0, 0, 15, 0.01));
  EXPECT_FALSE(detector.update(40, 0, 15, 0.006));
  EXPECT_FALSE(detector.update(0, 40, 15, 0.006));
  EXPECT_FALSE(detector.update(40, 0, 15, 0.006));
  EXPECT_FALSE(detector.update(19, 19, 15, 0.1));
  EXPECT_FALSE(detector.update(40, 40, 15, 0.006));
  EXPECT_TRUE(detector.update(40, 40, 15, 0.006));
}

TEST(JumpContactDetector, ResetRequiresNewFlight)
{
  JumpContactDetector detector;
  EXPECT_FALSE(detector.update(0, 0, 15, 0.01));
  EXPECT_TRUE(detector.update(40, 0, 15, 0.01));
  detector.reset();
  EXPECT_FALSE(detector.update(40, 0, 15, 0.1));
  EXPECT_FALSE(detector.update(0, 0, 15, 0.01));
  EXPECT_TRUE(detector.update(0, 40, 15, 0.01));
}

TEST(JumpContactDetector, InvalidSamplesBreakContinuousConfirmation)
{
  JumpContactDetector detector;
  EXPECT_FALSE(detector.update(0, 0, 15, 0.01));
  EXPECT_FALSE(detector.update(40, 0, 15, 0.006));
  EXPECT_FALSE(detector.update(NAN, 0, 15, 0.01));
  EXPECT_FALSE(detector.update(40, 0, 15, 0.006));
  EXPECT_FALSE(detector.update(40, 0, 15, -0.01));
  EXPECT_FALSE(detector.update(40, 0, 15, 0.006));
  EXPECT_TRUE(detector.update(40, 0, 15, 0.006));
}
