#include "edu_robot/algorithm/odometry_kalman_filter_model.hpp"
#include <cstddef>

namespace eduart {
namespace robot {
namespace algorithm {

OdometryKalmanFilterModel::Matrix OdometryKalmanFilterModel::getPredictionMatrix(
  const StateVector& current_state, const double delta_time) const
{
  enum Index : Eigen::Index {
    POS_X    = AttributePack::index<kinematic::Attribute::POS_X>(),
    POS_Y    = AttributePack::index<kinematic::Attribute::POS_Y>(),
    VEL_X    = AttributePack::index<kinematic::Attribute::VEL_X>(),
    VEL_Y    = AttributePack::index<kinematic::Attribute::VEL_Y>(),
    YAW      = AttributePack::index<kinematic::Attribute::YAW>(),
    YAW_RATE = AttributePack::index<kinematic::Attribute::YAW_RATE>(),
  };

  Matrix prediction_matrix = Matrix::Identity();

  // prediction_matrix(POS_X, VEL) = delta_time * std::cos(current_state.yaw());
  // prediction_matrix(POS_X, ACC) = 0.5 * delta_time * delta_time * std::cos(current_state.yaw());

  // prediction_matrix(POS_Y, VEL) = delta_time * std::sin(current_state.yaw());
  // prediction_matrix(POS_Y, ACC) = 0.5 * delta_time * delta_time * std::sin(current_state.yaw());

  // prediction_matrix(VEL, ACC) = delta_time;

  // prediction_matrix(YAW, YAW_RATE) = delta_time;
  // prediction_matrix(POS_X, YAW_RATE) =
  //   delta_time * delta_time * 0.5 * -std::sin(current_state.yaw()) * delta_time * current_state.velocity();
  // prediction_matrix(POS_Y, YAW_RATE) = 
  //   delta_time * delta_time * 0.5 *  std::cos(current_state.yaw()) * delta_time * current_state.velocity();

  return prediction_matrix;
}

OdometryKalmanFilterModel::Matrix OdometryKalmanFilterModel::getSystemNoiseMatrix(
  const StateVector& current_state, const double delta_time) const
{
  Matrix noise_matrix = Matrix::Zero();

  // // add jerk system noise
  // {
  //   constexpr double jerk_variance = 0.5 * 0.5;
  //   StateVector noise_variances;

  //   noise_variances.x() = (delta_time * delta_time * delta_time / 6.0) * std::cos(current_state.yaw());
  //   noise_variances.y() = (delta_time * delta_time * delta_time / 6.0) * std::sin(current_state.yaw());
  //   noise_variances.velocity() = 0.5 * delta_time * delta_time;
  //   noise_variances.acceleration() = delta_time;

  //   const StateVector::Vector noise_vector(static_cast<StateVector::Vector>(noise_variances));
  //   noise_matrix += jerk_variance * noise_vector * noise_vector.transpose();
  // }
  // // add yaw acceleration noise
  // {
  //   constexpr double yaw_acceleration_variance = base::Angle::createFromDegree(720) * base::Angle::createFromDegree(720);
  //   StateVector noise_variances;

  //   noise_variances.yaw() = 0.5 * delta_time * delta_time;
  //   noise_variances.yawRate() = delta_time;

  //   const StateVector::Vector noise_vector(static_cast<StateVector::Vector>(noise_variances));
  //   noise_matrix += yaw_acceleration_variance * noise_vector * noise_vector.transpose();
  // }

  return noise_matrix;
}

} // end namespace algorithm
} // end namespace robot
} // end namespace eduart
