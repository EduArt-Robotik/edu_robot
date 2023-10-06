/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <deque>
#include <numeric>
#include <cmath>

namespace eduart {
namespace robot {
namespace diagnostic {

template <typename Type>
class StandardDeviation
{
public:
  StandardDeviation(const std::size_t queue_size)
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
    _variance = static_cast<Type>(0);

    for (const auto value : _data) {
      _variance += (value - _mean) * (value - _mean);
    }

    _variance /= static_cast<Type>(_data.size());
    _std_deviation = std::sqrt(_variance);
  }

  inline Type mean() const { return _mean; }
  inline Type variance() const { return _variance; }
  inline Type stdDeviation() const { return _std_deviation; }

private:
  std::size_t _queue_size;
  std::deque<Type> _data;
  Type _mean = static_cast<Type>(0);
  Type _variance = static_cast<Type>(0);
  Type _std_deviation = static_cast<Type>(0);
};

} // end namespace diagnostic
} // end namespace robot
} // end namespace eduart
