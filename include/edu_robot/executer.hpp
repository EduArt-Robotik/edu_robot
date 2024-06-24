/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace eduart {
namespace robot {

using namespace std::chrono_literals;

class Job
{
public:
  Job() = default;
  Job(std::function<void()> do_job_function, const std::chrono::microseconds time_interval)
    : _do_job(do_job_function)
    , _stamp_last_done(std::chrono::system_clock::now())
    , _time_interval(time_interval)
  {
    _is_active = isValid();
  }

  inline bool isValid() const { return !(_do_job == nullptr); }
  inline bool isActive() const { return _is_active; }
  inline std::chrono::time_point<std::chrono::system_clock> stampLastDone() const { return _stamp_last_done; }
  inline bool shouldItBeDone(
    const std::chrono::time_point<std::chrono::system_clock> stamp_now = std::chrono::system_clock::now()) const
  {
    return (stamp_now - _stamp_last_done) > _time_interval;
  }
  void operator()() {
    if (isActive() == false) {
      return;
    }

    _do_job();
    _stamp_last_done = std::chrono::system_clock::now();
  }

  void activate() { _is_active = true; }
  void deactivate() { _is_active = false; }

private:
  bool _is_active = false;
  std::function<void()> _do_job;
  std::chrono::time_point<std::chrono::system_clock> _stamp_last_done;
  std::chrono::microseconds _time_interval = 10ms;
};

class Executer
{
public:
  Executer(const std::chrono::microseconds max_time_between_runs = 1ms);
  ~Executer();

  void start();
  void addJob(std::function<void()> do_job_function, const std::chrono::microseconds time_interval);
  void stop();

private:
  void run();

  std::atomic_bool _is_running = false;
  std::chrono::microseconds _max_time_between_runs = 1ms;
  std::chrono::time_point<std::chrono::system_clock> _stamp_last_run;
  std::thread _executer;
  std::mutex _mutex;
  std::vector<Job> _jobs;
};

} // end namespace eduart
} // end namespace robot