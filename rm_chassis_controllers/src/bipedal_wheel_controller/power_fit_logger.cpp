#include "rm_chassis_controllers/bipedal_wheel_controller/power_fit_logger.h"
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <iomanip>

namespace rm_chassis_controllers
{
PowerFitLogger::PowerFitLogger(const std::string& directory, const std::string& session)
{
  try
  {
    boost::filesystem::create_directories(directory);
    const auto path = boost::filesystem::path(directory) / (session + ".csv");
    file_.open(path.string(), std::ios::out);
    if (!file_)
      throw std::runtime_error("cannot open CSV");
    file_ << "stamp,session_id,model_version,last_solve_stamp,sample_count,model_valid,metrics_valid,r2_valid,"
             "last_solve_success,model_accepted,lambda,alpha_ridge,rmse,r2,bias";
    for (int i = 0; i < 12; ++i)
      file_ << ",c" << i;
    file_ << '\n' << std::flush;
    if (!file_)
      throw std::runtime_error("cannot write CSV header");
    ROS_INFO_STREAM("[PowerID] CSV: " << path.string());
    worker_ = std::thread(&PowerFitLogger::run, this);
  }
  catch (const std::exception& e)
  {
    failed_ = true;
    ROS_ERROR_STREAM("[PowerID] CSV disabled: " << e.what());
  }
}
PowerFitLogger::~PowerFitLogger()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  ready_.notify_one();
  if (worker_.joinable())
    worker_.join();
}
void PowerFitLogger::enqueue(const rm_msgs::PowerModelFit& snapshot)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (failed_ || stopping_)
    return;
  if (queue_.size() >= 256)
  {
    ROS_ERROR_THROTTLE(2.0, "[PowerID] CSV queue full; dropping snapshot");
    return;
  }
  queue_.push_back(snapshot);
  ready_.notify_one();
}
void PowerFitLogger::run()
{
  for (;;)
  {
    rm_msgs::PowerModelFit s;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      ready_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
      if (queue_.empty() && stopping_)
        break;
      s = queue_.front();
      queue_.pop_front();
    }
    file_ << std::setprecision(17) << s.header.stamp.toSec() << ',' << s.session_id << ',' << s.model_version << ','
          << s.last_solve_stamp.toSec() << ',' << s.sample_count << ',' << s.model_valid << ',' << s.metrics_valid
          << ',' << s.r2_valid << ',' << s.last_solve_success << ',' << s.model_accepted << ',' << s.lambda << ','
          << s.alpha_ridge << ',' << s.rmse << ',' << s.r2 << ',' << s.bias;
    for (double c : s.c)
      file_ << ',' << c;
    file_ << '\n' << std::flush;
    if (!file_)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      failed_ = true;
      queue_.clear();
      ROS_ERROR("[PowerID] CSV write failed; logging disabled");
      break;
    }
  }
}
}  // namespace rm_chassis_controllers
