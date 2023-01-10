/**
 * This file defines a movement model for a ego vehicle using a kalman filter.
 * Origin idea is from https://github.com/franc0r/libfrancor/blob/master/francor_mapping/include/francor_mapping/kalman_filter_model.h
 *
 * \author Christian Wendt (knueppl@gmx.de)
 * \date January 2023
 */
#pragma once

#include "edu_robot/algorithm/kinematic_state_vector.hpp"

#include <Eigen/Core>

namespace eduart {
namespace robot {
namespace algorithm {

/**
 * \brief This class is the base class for kalman filter models. A model defines the state vector and
 *        provides functionality to predict this state.
 */

// prototype declaration
template <class StateVectorType>
class KalmanFilterModel;

// specialization for KinematicStateVector
template <kinematic::Attribute... Attributes>
class KalmanFilterModel<kinematic::AttributePack<Attributes...>>
{
public:
  using StateVector = kinematic::StateVector<kinematic::AttributePack<Attributes...>>;
  static constexpr std::size_t dimension = StateVector::count();
  using Matrix = Eigen::Matrix<double, dimension, dimension>;

  KalmanFilterModel() { }

  /**
   * \brief Provides a matrix that can be used to predict in time the current state.
   * \param delta_time Is delta t. The state wil be predicted by this time step.
   * \return Prediction matrix with dimension of this model.
   */
  virtual Matrix getPredictionMatrix(const StateVector& current_state, const double delta_time) const = 0;

  /**
   * \brief Creates the system noise covariance matrix based on delta t.
   * \param delta_time Is delta t. The created noise covariance matrix will reflect this time period.
   * \return System noise covariance matrix.
   */
  virtual Matrix getSystemNoiseMatrix(const StateVector& current_state, const double delta_time) const = 0;
};

} // end namespace algorithm
} // end namespace robot
} // end namespace eduart
