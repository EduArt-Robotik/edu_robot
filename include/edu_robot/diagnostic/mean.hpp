/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/diagnostic/diagnostic_check.hpp"

#include <deque>
#include <numeric>

namespace eduart {
namespace robot {
namespace diagnostic {

template <typename Type>
class Mean
{
public:
  Mean(const std::size_t queue_size)
    : _queue_size(queue_size)
  { }

  void update(const Type value)
  {
    _data.push_back(value);

    // maintain the queue size
    while (_data.size() > _queue_size) {
      _data.pop_front();
    }

    _mean = std::accumulate(_data.begin(), _data.end(), static_cast<Type>(0)) / static_cast<Type>(_data.size());
  }

  inline Type mean() const { return _mean; }

protected:
  std::size_t _queue_size;
  std::deque<Type> _data;
  Type _mean = static_cast<Type>(0);
};

template <typename Type, class Checker>
class MeanDiagnostic : public Mean<Type>
                     , public DiagnosticCheckList
{
public:
  MeanDiagnostic(
    const std::string& name, const std::string& unit, const std::size_t queue_size, const Type level_warning_mean,
    const Type level_error_mean)
  : Mean<Type>(queue_size)
  , _checker_mean(name + " (" + unit + ")", level_warning_mean, level_error_mean)
  { }

  void addToLevelList(LevelList& list) const override
  {
    list.emplace(_checker_mean.name(), _checker_mean.estimateLevel(Mean<Type>::mean()));
  }
  void addToEntryList(EntryList& list) const override
  {
    list.emplace(_checker_mean.name(), std::to_string(Mean<Type>::mean()));
  }

private:
  DiagnosticCheck<Type, Checker> _checker_mean;
};

} // end namespace diagnostic
} // end namespace robot
} // end namespace eduart
