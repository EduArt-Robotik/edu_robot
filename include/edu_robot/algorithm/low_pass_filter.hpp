/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/angle.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <Eigen/Geometry>

#include <array>
#include <tuple>
#include <utility>
#include <type_traits>

namespace eduart {
namespace robot {
namespace algorithm {
namespace impl {

template <typename Type, typename ExchangeType>
struct LowPassFilterOperation;

template <typename DataType>
struct LowPassFilterOperation<DataType, std::enable_if_t<std::is_floating_point_v<DataType>, DataType>> {
  static inline DataType update(const DataType input, const DataType previous_value, const float filter_weight) {
    return (1.0f - filter_weight) * previous_value + filter_weight * input;
  }
  static inline void clear(DataType& value) {
    value = static_cast<DataType>(0);
  }
  static inline DataType getValue(const DataType& value) { return value; } 
};
template <>
struct LowPassFilterOperation<decltype(geometry_msgs::msg::Pose::position),
                                     decltype(geometry_msgs::msg::Pose::position)> {
  using DataType = decltype(geometry_msgs::msg::Pose::position);

  static inline DataType update(const DataType& input, const DataType& previous_value, const float filter_weight) {
    // \todo actually the operation above (floating point) should be used here.
    DataType updated_value;
    updated_value.x = static_cast<double>(1.0f - filter_weight) * previous_value.x + static_cast<double>(filter_weight) * input.x;
    updated_value.y = static_cast<double>(1.0f - filter_weight) * previous_value.y + static_cast<double>(filter_weight) * input.y;
    updated_value.z = static_cast<double>(1.0f - filter_weight) * previous_value.z + static_cast<double>(filter_weight) * input.z;

    return updated_value;
  }
  static inline void clear(DataType& value) {
    value.x = 0.0;
    value.y = 0.0;
    value.z = 0.0;
  }
  static inline DataType getValue(const DataType& value) { return value; } 
};
template <>
struct LowPassFilterOperation<Eigen::Quaternionf, decltype(geometry_msgs::msg::Pose::orientation)> {
  static Eigen::Quaternionf update(
    const decltype(geometry_msgs::msg::Pose::orientation)& input, const Eigen::Quaternionf& previous_value,
    const float filter_weight)
  {
    return previous_value.slerp(filter_weight, Eigen::Quaternionf(input.w, input.x, input.y, input.z));
  }
  static inline void clear(Eigen::Quaternionf& value) {
    value = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
  }
  static inline decltype(geometry_msgs::msg::Pose::orientation) getValue(const Eigen::Quaternionf& value) {
    decltype(geometry_msgs::msg::Pose::orientation) orientation;
    orientation.w = value.w();
    orientation.x = value.x();
    orientation.y = value.y();
    orientation.z = value.z();

    return orientation;
  }
};

} // end namespace impl

template <typename Type, typename ExchangeType = Type>
class LowPassFiler
{
public:
  using data_type = Type;
  using exchange_type = ExchangeType;

  struct Parameter {
    Type filter_weight = 1.0;
  };

  LowPassFiler(const Parameter& parameter) : _parameter(parameter) { }

  inline void clear() { impl::LowPassFilterOperation<Type, ExchangeType>::clear(_value); }
  inline ExchangeType getValue() const {
    return impl::LowPassFilterOperation<Type, ExchangeType>::getValue(_value);
  }
  inline Type operator()(const Type& value) {
    update(value, _parameter.filter_weight);
    return getValue();
  }

protected:
  void update(const ExchangeType& input, const float filter_weight) {
    _value = impl::LowPassFilterOperation<Type, ExchangeType>::update(
      input, _value, filter_weight
    );
  }

  Parameter _parameter;
  Type _value;
};

} // end namespace algorithm
} // end namespace robot
} // end namespace eduart
