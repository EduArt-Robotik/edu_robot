#include "edu_robot/executer.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <thread>

namespace eduart {
namespace robot {

Executer::Executer(const std::chrono::microseconds max_time_between_runs)
  : _max_time_between_runs(max_time_between_runs)
{

}

Executer::~Executer()
{
  stop();
}

void Executer::start()
{
  _is_running = true;
  _stamp_last_run = std::chrono::system_clock::now();
  _executer = std::thread(&Executer::run, this);
}

void Executer::stop()
{
  _is_running = false;
  _executer.join();
}

void Executer::addJob(std::function<void()> do_job_function, const std::chrono::microseconds time_interval)
{
  if (_is_running) {
    RCLCPP_ERROR(rclcpp::get_logger("Executer"), "can't add job when executer is running.");
    return;
  }

  Job job(do_job_function, time_interval);

  if (job.isValid() == false) {
    RCLCPP_ERROR(rclcpp::get_logger("Executer"), "can't add job. Given function or time interval is not valid.");
    return;
  }

  _jobs.push_back(job);
}

void Executer::run()
{
  while (_is_running) {
    // process all jobs if nothing to do sleep
    bool did_something = false;
    const auto stamp_now = std::chrono::system_clock::now();

    for (auto& job : _jobs) {
      if (job.shouldItBeDone(stamp_now) == false || job.isActive() == false) {
        // job should not be done --> try next job
        continue;
      }

      // do job
      job();
      did_something = true;
    }

    if (did_something == false) {
      // all jobs are done --> sleep until next operation
      const auto diff = stamp_now - _stamp_last_run;
      const auto wait_time = diff > _max_time_between_runs ? _max_time_between_runs : _max_time_between_runs - diff;
      std::this_thread::sleep_for(wait_time);
      return;
    }

    _stamp_last_run = stamp_now;
  }
}

} // end namespace eduart
} // end namespace robot
