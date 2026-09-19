#include <gtest/gtest.h>
#include <rm_chassis_controllers/bipedal_wheel_controller/power_fit_logger.h>
#include <boost/filesystem.hpp>
#include <sstream>
#include <ros/ros.h>

TEST(PowerFitLogger, DrainsAndPreservesSnapshot)
{
  const auto dir = boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("power-fit-%%%%%%%%");
  rm_msgs::PowerModelFit s;
  s.session_id = "test";
  s.header.stamp = ros::Time(123, 500000000);
  s.model_version = 7;
  s.sample_count = 99;
  s.model_valid = true;
  s.metrics_valid = true;
  s.r2_valid = true;
  s.last_solve_success = true;
  s.model_accepted = true;
  s.lambda = 0.999;
  s.alpha_ridge = 0.001;
  s.rmse = 1.23456789012345;
  s.r2 = 0.91;
  s.bias = -0.25;
  for (size_t i = 0; i < s.c.size(); ++i)
    s.c[i] = 0.123456789012345 + i;
  {
    rm_chassis_controllers::PowerFitLogger logger(dir.string(), s.session_id);
    logger.enqueue(s);
    s.model_version = 8;
    logger.enqueue(s);
  }
  std::ifstream f((dir / "test.csv").string());
  std::string line;
  ASSERT_TRUE(bool(std::getline(f, line)));
  EXPECT_NE(line.find("c0,c1"), std::string::npos);
  for (int version = 7; version <= 8; ++version)
  {
    ASSERT_TRUE(bool(std::getline(f, line)));
    std::stringstream row(line);
    std::vector<std::string> fields;
    std::string field;
    while (std::getline(row, field, ','))
      fields.push_back(field);
    ASSERT_EQ(fields.size(), 27u);
    EXPECT_DOUBLE_EQ(std::stod(fields[0]), 123.5);
    EXPECT_EQ(std::stoi(fields[2]), version);
    EXPECT_DOUBLE_EQ(std::stod(fields[12]), s.rmse);
    for (size_t i = 0; i < 12; ++i)
      EXPECT_DOUBLE_EQ(std::stod(fields[15 + i]), s.c[i]);
  }
  EXPECT_FALSE(bool(std::getline(f, line)));
  boost::filesystem::remove_all(dir);
}
TEST(PowerFitLogger, InvalidDirectoryDoesNotThrow)
{
  ros::Time::init();
  rm_chassis_controllers::PowerFitLogger logger("/dev/null/invalid", "test");
  logger.enqueue(rm_msgs::PowerModelFit());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
