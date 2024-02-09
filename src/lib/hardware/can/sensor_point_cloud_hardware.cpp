#include "edu_robot/hardware/can/sensor_point_cloud_hardware.hpp"
#include "edu_robot/hardware/can/message_definition.hpp"
#include "edu_robot/hardware/can/can_rx_data_endpoint.hpp"


namespace eduart {
namespace robot {
namespace hardware {
namespace can {

using can::CanRxDataEndPoint;
using can::message::sensor::tof::StartMeasurement;
using can::message::sensor::tof::MeasurementComplete;
using can::message::sensor::tof::ZoneMeasurement;

SensorPointCloudHardware::Parameter
SensorPointCloudHardware::get_parameter(const std::string& name, const Parameter& default_parameter, rclcpp::Node& ros_node)
{
  SensorPointCloudHardware::Parameter parameter;

  ros_node.declare_parameter<int>(name + ".can_id.trigger", default_parameter.can_id.trigger);
  ros_node.declare_parameter<int>(name + ".can_id.complete", default_parameter.can_id.complete);
  ros_node.declare_parameter<int>(name + ".can_id.measurement", default_parameter.can_id.measurement);

  parameter.can_id.trigger = ros_node.get_parameter(name + ".can_id.trigger").as_int();
  parameter.can_id.complete = ros_node.get_parameter(name + ".can_id.complete").as_int();
  parameter.can_id.measurement = ros_node.get_parameter(name + ".can_id.measurement").as_int();

  return parameter;
}

SensorPointCloudHardware::SensorPointCloudHardware(
  const std::string& name, const Parameter& parameter, rclcpp::Node& ros_node, std::shared_ptr<Communicator> communicator)
  : SensorPointCloud::SensorInterface()
  , CanGatewayTxRxDevice(communicator)
  , _parameter(parameter)
  , _ros_node(ros_node)
{
  (void)name;
  auto measurement_end_point = CanRxDataEndPoint::make_data_endpoint<ZoneMeasurement>(
    _parameter.can_id.measurement,
    std::bind(&SensorPointCloudHardware::processRxData, this, std::placeholders::_1));
  _communicator->registerRxDataEndpoint(std::move(measurement_end_point));
}

void SensorPointCloudHardware::processRxData(const message::RxMessageDataBuffer& data)
{
  if (_callback_process_measurement == nullptr) {
    return;
  }

  _callback_process_measurement(
    ZoneMeasurement::zone(data),
    ZoneMeasurement::distance(data),
    ZoneMeasurement::sigma(data)
  );
}

void SensorPointCloudHardware::initialize(const SensorPointCloud::Parameter& parameter)
{
  _processing_data.frame_number = 0;

  _timer_get_measurement = _ros_node.create_wall_timer(
    parameter.measurement_interval, std::bind(&SensorPointCloudHardware::processMeasurement, this)
  );
}

void SensorPointCloudHardware::processMeasurement()
{
  try {
    // Get measurement data from can gateway and parse it to processing pipeline.
    auto request = Request::make_request<StartMeasurement>(
      _parameter.can_id.trigger, _processing_data.frame_number, 0xff);

    _processing_data.future_response = _communicator->sendRequest(std::move(request));
    _processing_data.frame_number++;
  }
  catch (std::exception& ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SensorPointCloudHardware"),
      "error occurred during processing measurement. what = %s.", ex.what()
    );
  }
}

} // end namespace can
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
