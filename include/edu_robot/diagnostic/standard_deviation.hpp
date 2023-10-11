/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/diagnostic/diagnostic_check.hpp"
#include "edu_robot/diagnostic/mean.hpp"

#include <deque>
#include <numeric>
#include <cmath>

namespace eduart {
namespace robot {
namespace diagnostic {

template <typename Type>
class StandardDeviation : public Mean<Type>
{
public:
  StandardDeviation(const std::size_t queue_size)
    : Mean<Type>(queue_size)
  { }

  void update(const Type value)
  {
    Mean<Type>::update(value);

    _variance = static_cast<Type>(0);

    for (const auto value : Mean<Type>::_data) {
      _variance += (value - Mean<Type>::_mean) * (value - Mean<Type>::_mean);
    }

    _variance /= static_cast<Type>(Mean<Type>::_data.size());
    _std_deviation = std::sqrt(_variance);
  }

  inline Type variance() const { return _variance; }
  inline Type stdDeviation() const { return _std_deviation; }

private:
  Type _variance = static_cast<Type>(0);
  Type _std_deviation = static_cast<Type>(0);
};

template <typename Type, class Checker>
class StandardDeviationDiagnostic : public StandardDeviation<Type>
                                  , public DiagnosticCheckList
{
public:
  StandardDeviationDiagnostic(
    const std::string& name, const std::string& unit, const std::size_t queue_size, const Type level_warning_mean,
    const Type level_error_mean, const Type level_warning_std_dev, const Type level_error_std_dev)
  : StandardDeviation<Type>(queue_size)
  , _checker_mean(name + " (" + unit + ")", level_warning_mean, level_error_mean)
  , _checker_std_deviation(name + " (" + unit + ") std dev", level_warning_std_dev, level_error_std_dev)
  { }

  void addToLevelList(LevelList& list) const override
  {
    list.emplace(_checker_mean.name(), _checker_mean.estimateLevel(StandardDeviation<Type>::mean()));
    list.emplace(
      _checker_std_deviation.name(),
      _checker_std_deviation.estimateLevel(StandardDeviation<Type>::stdDeviation())
    );
  }
  void addToEntryList(EntryList& list) const override
  {
    list.emplace(_checker_mean.name(), std::to_string(StandardDeviation<Type>::mean()));
    list.emplace(_checker_std_deviation.name(), std::to_string(StandardDeviation<Type>::stdDeviation()));
  }

private:
  std::string _name;
  DiagnosticCheck<Type, Checker> _checker_mean;
  DiagnosticCheck<Type, Checker> _checker_std_deviation;
};

} // end namespace diagnostic
} // end namespace robot
} // end namespace eduart
