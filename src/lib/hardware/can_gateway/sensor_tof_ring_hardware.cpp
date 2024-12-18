#include "edu_robot/hardware/can_gateway/sensor_tof_ring_hardware.hpp"
#include "edu_robot/hardware/can_gateway/can/can_rx_data_endpoint.hpp"
#include "edu_robot/hardware/can_gateway/can/message_definition.hpp"
#include "edu_robot/hardware/can_gateway/can/can_request.hpp"

#include <edu_robot/hardware/communicator_node.hpp>

#include <functional>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

using can::Request;
using can::message::sensor::tof::StartMeasurement;
using can::message::sensor::tof::MeasurementComplete;
using can::message::sensor::tof::TriggerDataTransmission;

static std::shared_ptr<sensor_msgs::msg::PointCloud2> create_point_cloud(
  const SensorTofRingHardware::Parameter& parameter)
{
  std::size_t number_of_points = 0;

  for (const auto& sensor : parameter.tof_sensor) {
    number_of_points += sensor.parameter.number_of_zones.horizontal * sensor.parameter.number_of_zones.vertical;
  }

  // Preparing Point Cloud
  auto point_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  point_cloud->header.frame_id = "unkown";
  point_cloud->width = 0;
  point_cloud->height = 1;
  point_cloud->is_bigendian = false;
  point_cloud->point_step = 4 * 4; // 4 * float
  point_cloud->row_step = point_cloud->point_step * point_cloud->width;
  point_cloud->data.reserve(number_of_points * point_cloud->point_step);

  sensor_msgs::msg::PointField point_field;
  point_field.name = "x";
  point_field.offset = 0;
  point_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_field.count = 1;
  point_cloud->fields.push_back(point_field);

  point_field.name = "y";
  point_field.offset = 4;
  point_cloud->fields.push_back(point_field);

  point_field.name = "z";
  point_field.offset = 8;
  point_cloud->fields.push_back(point_field);

  point_field.name = "sigma";
  point_field.offset = 12;
  point_cloud->fields.push_back(point_field);

  return point_cloud;
}

SensorTofRingHardware::Parameter SensorTofRingHardware::get_parameter(
  const std::string& name, const std::vector<std::string>& sensor_names, rclcpp::Node& ros_node)
{
  Parameter parameter;
  SensorTofHardware::Parameter sensor_parameter;
  sensor_parameter.trigger_measurement = false; // disable triggering of new measurement in sub sensor

  for (const auto& sensor_name : sensor_names) {
    parameter.tof_sensor.push_back({
      SensorTofHardware::get_parameter(
        name + '.' + sensor_name, sensor_parameter, ros_node
      ),
      sensor_name,
      Sensor::get_transform_from_parameter(name + '.' + sensor_name, ros_node),
    });
  }

  ros_node.declare_parameter<int>(name + ".measurement_interval", parameter.measurement_interval.count());
  ros_node.declare_parameter<int>(name + ".can_id.trigger", parameter.can_id_trigger);
  ros_node.declare_parameter<int>(name + ".can_id.measurement_complete", parameter.can_id_measurement_complete);

  parameter.measurement_interval = std::chrono::milliseconds(
    ros_node.get_parameter(name + ".measurement_interval").as_int());
  parameter.can_id_trigger = ros_node.get_parameter(name + ".can_id.trigger").as_int();
  parameter.can_id_measurement_complete = ros_node.get_parameter(name + ".can_id.measurement_complete").as_int();

  return parameter;
}

SensorTofRingHardware::SensorTofRingHardware(
  const std::string& name, const Parameter& parameter, rclcpp::Node& ros_node, std::shared_ptr<Executer> executer,
  std::shared_ptr<Communicator> communicator)
  : _parameter(parameter)
  , _ros_node(ros_node)
  , _communication_node(std::make_shared<CommunicatorNode>(executer, communicator))
{
  (void)name;

  for (std::size_t i = 0; i < _parameter.tof_sensor.size(); ++i) {
    _sensor.emplace_back(std::make_shared<SensorTofHardware>(
      _parameter.tof_sensor[i].name, _parameter.tof_sensor[i].parameter, _ros_node, executer, communicator
    ));
    _processing_data.sensor_activation_bits |= (1 << (_parameter.tof_sensor[i].parameter.sensor_id - 1));
    _sensor.back()->registerCallbackProcessMeasurementData(
      std::bind(&SensorTofRingHardware::processPointcloudMeasurement, this, std::placeholders::_1, i)
    );
  }
}

void SensorTofRingHardware::initialize(const SensorPointCloud::Parameter& parameter)
{
  for (auto& tof_sensor : _sensor) {
    tof_sensor->initialize(parameter);
  }

  _processing_data.frame_number = 0;
  _processing_data.point_cloud = create_point_cloud(_parameter);
  _processing_data.point_cloud->width = 0;
  _processing_data.point_cloud->height = 1;
  _processing_data.received_points.resize(_parameter.number_sensors(), false);

  _communication_node->createRxDataEndPoint<can::CanRxDataEndPoint, MeasurementComplete>(
    _parameter.can_id_measurement_complete, std::bind(&SensorTofRingHardware::processFinishMeasurement, this, std::placeholders::_1), 8
  );
  _communication_node->addSendingJob(
    std::bind(&SensorTofRingHardware::processStartMeasurement, this),
    _parameter.measurement_interval
  );
}

void SensorTofRingHardware::processStartMeasurement()
{
  if (_processing_data.active_measurement != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SensorTofRingHardware"),
      "current measurements no finished! Data cloud be corrupt! active = %u", _processing_data.active_measurement
    );
    RCLCPP_INFO(rclcpp::get_logger("SensorTofRingHardware"), "publishing pointcloud anyway.");
    _callback_process_measurement(*_processing_data.point_cloud);
  }

  try {
    // Get measurement data from can gateway and parse it to processing pipeline.
    auto request = Request::make_request<StartMeasurement>(
      _parameter.can_id_trigger, _processing_data.frame_number, _processing_data.sensor_activation_bits
    );

    _communication_node->sendRequest(std::move(request), 100ms);

    _processing_data.point_cloud->header.stamp = _ros_node.get_clock()->now();
    _processing_data.point_cloud->data.clear();
    _processing_data.point_cloud->width = 0;
    _processing_data.point_cloud->height = 1;
    _processing_data.frame_number++;
    _processing_data.active_measurement = _processing_data.sensor_activation_bits;
    std::fill(_processing_data.received_points.begin(), _processing_data.received_points.end(), false);
  }
  catch (std::exception& ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SensorTofRingHardware"),
      "error occurred during start measurement. what = %s.", ex.what()
    );
  }
}

void SensorTofRingHardware::processFinishMeasurement(const message::RxMessageDataBuffer& rx_buffer)
{
  // mark that given sensor finished measurement
  _processing_data.active_measurement &= ~(1 << (MeasurementComplete::sensor(rx_buffer) - 1));

  if (_processing_data.active_measurement != 0) {
    // measurement not completed yet --> return and wait
    return;
  }

  // measurement completed --> trigger data transmission
  try {
    auto request = Request::make_request<TriggerDataTransmission>(
      _parameter.can_id_trigger, _processing_data.sensor_activation_bits
    );
    _communication_node->sendRequest(std::move(request), 100ms);
  }
  catch (std::exception& ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SensorTofRingHardware"),
      "error occurred during trigger data transmission. what = %s.", ex.what()
    );
  }
}

void SensorTofRingHardware::processPointcloudMeasurement(
  sensor_msgs::msg::PointCloud2& point_cloud, const std::size_t sensor_index)
{
  // Note: expect the given point cloud is of exact same type as sensor point cloud.

  // Plausibility Check
  // if (sensor_index != _processing_data.next_expected_sensor) {
  //   RCLCPP_ERROR(
  //     rclcpp::get_logger("SensorTofRingHardware"),
  //     "sensor_index %u is not the expected one (%u). The point cloud will contain corrupted data.",
  //     static_cast<unsigned int>(sensor_index),
  //     static_cast<unsigned int>(_processing_data.next_expected_sensor)
  //   );
  // }

  // Transform given point cloud into sensor frame.
  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(_parameter.tof_sensor[sensor_index].transform);
  sensor_msgs::msg::PointCloud2 point_cloud_transformed;
  tf2::doTransform(point_cloud, point_cloud_transformed, transform);

  // Copy given point cloud into sensor point cloud.
  _processing_data.point_cloud->width += point_cloud.width * point_cloud.height;
  _processing_data.point_cloud->data.insert(
    _processing_data.point_cloud->data.end(),
    point_cloud_transformed.data.begin(),
    point_cloud_transformed.data.end()
  );
  _processing_data.received_points[sensor_index] = true;

  for (const auto received : _processing_data.received_points) {
    if (received == false) {
      // Measurement no finished --> do nothing
      return;
    }
  }

  // Measurement finished --> publish point cloud.
  _callback_process_measurement(*_processing_data.point_cloud);
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
