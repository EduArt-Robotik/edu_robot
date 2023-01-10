/**
 * Defines a Kalman filter model for odometry data fusion.
 *
 * \author Christian Wendt (christian.wendt@eduart-robotik.com)
 * \date January 2023
 */
#pragma once

#include "edu_robot/algorithm/kalman_filter_model.hpp"

namespace eduart {
namespace robot {
namespace algorithm {

using OdometryKalmanFilterAttributes = kinematic::AttributePack<kinematic::Attribute::POS_X,
                                                                kinematic::Attribute::POS_Y,
                                                                kinematic::Attribute::VEL_X,
                                                                kinematic::Attribute::VEL_Y,
                                                                kinematic::Attribute::YAW,
                                                                kinematic::Attribute::YAW_RATE>;

class OdometryKalmanFilterModel : public KalmanFilterModel<OdometryKalmanFilterAttributes>
{
public:
  Matrix getPredictionMatrix(const StateVector& current_state, const double delta_time) const override;
  Matrix getSystemNoiseMatrix(const StateVector& current_state, const double delta_time) const override;
};

} // end namespace algorithm
} // end namespace robot
} // end namespace eduart
