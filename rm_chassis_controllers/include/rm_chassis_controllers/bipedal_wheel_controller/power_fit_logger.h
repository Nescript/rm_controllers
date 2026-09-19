#pragma once
#include <rm_msgs/PowerModelFit.h>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <thread>

namespace rm_chassis_controllers
{
// Only used by the non-RT power callback. Destruction drains accepted snapshots.
class PowerFitLogger
{
public:
  PowerFitLogger(const std::string& directory, const std::string& session);
  ~PowerFitLogger();
  void enqueue(const rm_msgs::PowerModelFit& snapshot);

private:
  void run();
  std::ofstream file_;
  std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<rm_msgs::PowerModelFit> queue_;
  bool stopping_ = false, failed_ = false;
  std::thread worker_;
};
}  // namespace rm_chassis_controllers
