/**
 * This file defines a kinematic state vector that is constructed by given attributes.
 * \author Christian Wendt (knueppl@gmx.de)
 * \date 23. December 2022
 */
#pragma once

#include "edu_robot/algorithm/kinematic_attribute.hpp"
#include <cstddef>
#include <type_traits>

namespace eduart {
namespace robot {
namespace algorithm {
namespace kinematic {

template <Attribute Target>
struct resolve_type { using type = double; };

/**
 * \brief Represents a state vector for a Kalman filter. Each vector elements have a specific type. This class provides
 *        functionality to handle its data safely (for example angle normalization).
 */
template <typename>
struct StateVector;

template <Attribute... Attributes>
struct StateVector<AttributePack<Attributes...>>
{
  template <Attribute Target>
  typename resolve_type<Target>::type get() const {
    return std::get<AttributePack<Attributes...>::template index<Target>()>(_data);
  }
  template <Attribute Target>
  typename resolve_type<Target>::type set(const typename resolve_type<Target>::type value) {
    return std::get<AttributePack<Attributes...>::template index<Target>()>(_data) = value;
  }

  std::tuple<typename resolve_type<Attributes>::type...> _data;
};

} // end namespace kinematic
} // end namespace algorithm
} // end namespace robot
} // end namespace eduart
