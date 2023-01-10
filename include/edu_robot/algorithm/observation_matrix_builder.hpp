/**
 * Defines a observation matrix builder needed for Kalman filter.
 *
 * \author Christian Wendt (christian.wendt@eduart-robotik.com)
 * \date January 2023
 */
#pragma once

#include "edu_robot/algorithm/kinematic_attribute.hpp"

#include <Eigen/Core>
#include <Eigen/src/Core/util/Meta.h>
#include <cstddef>

namespace eduart {
namespace robot {
namespace algorithm {

template <class VectorA, class VectorB>
struct ObservationMatrixBuilder;

template <kinematic::Attribute... AttributesA, kinematic::Attribute... AttributesB>
struct ObservationMatrixBuilder<kinematic::AttributePack<AttributesA...>, kinematic::AttributePack<AttributesB...>>
{
  constexpr static std::size_t rows = sizeof...(AttributesA);
  constexpr static std::size_t cols = sizeof...(AttributesB);
  using MatrixType = Eigen::Matrix<double, rows, cols>;

  static MatrixType build()
  {
    using RowAttributes = kinematic::AttributePack<AttributesA...>;
    using ColAttributes = kinematic::AttributePack<AttributesB...>;
    MatrixType observation_matrix;

    for (Eigen::Index row = 0; row < observation_matrix.rows(); ++row) {
      for (Eigen::Index col = 0; col < observation_matrix.cols(); ++col) {
        observation_matrix(row, col) = (RowAttributes::attribute(row) == ColAttributes::attribute(col) ? 1 : 0);
      }
    }

    return observation_matrix;
  }
};

} // end namespace algorithm
} // end namespace robot
} // end namespace eduart
