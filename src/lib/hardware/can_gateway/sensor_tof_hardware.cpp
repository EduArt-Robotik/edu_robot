#include "edu_robot/hardware/can_gateway/sensor_tof_hardware.hpp"
#include "edu_robot/hardware/can_gateway/can/message_definition.hpp"
#include "edu_robot/hardware/can_gateway/can/can_rx_data_endpoint.hpp"
#include "edu_robot/hardware/can_gateway/can/can_request.hpp"
#include "edu_robot/hardware/can_gateway/sensor_virtual_range.hpp"

#include <edu_robot/hardware/communicator_node.hpp>

#include <rclcpp/logging.hpp>

#include <memory>
#include <limits>
#include <cstddef>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

using can::CanRxDataEndPoint;
using can::Request;
using can::message::sensor::tof::StartMeasurement;
using can::message::sensor::tof::ZoneMeasurement;
using can::message::sensor::tof::DataTransmissionComplete;
using can::message::sensor::tof::MeasurementComplete;

static std::shared_ptr<sensor_msgs::msg::PointCloud2> create_point_cloud(
  const SensorTofHardware::Parameter& parameter)
{
  // Preparing Point Cloud
  auto point_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  point_cloud->header.frame_id = "unkown";
  point_cloud->width = parameter.number_of_zones.horizontal;
  point_cloud->height = parameter.number_of_zones.vertical;
  point_cloud->is_bigendian = false;
  point_cloud->point_step = 4 * sizeof(float);
  point_cloud->row_step = point_cloud->point_step * point_cloud->width;
  point_cloud->data.resize(point_cloud->width * point_cloud->height * point_cloud->point_step);

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

static void set_nan_points(sensor_msgs::msg::PointCloud2& point_cloud)
{
  const std::size_t number_of_points = point_cloud.height * point_cloud.width;

  for (std::size_t point_idx = 0; point_idx < number_of_points; ++point_idx) {
    const std::size_t idx_point_x = point_idx * point_cloud.point_step + point_cloud.fields[0].offset;
    const std::size_t idx_point_y = point_idx * point_cloud.point_step + point_cloud.fields[1].offset;
    const std::size_t idx_point_z = point_idx * point_cloud.point_step + point_cloud.fields[2].offset;
    const std::size_t idx_sigma   = point_idx * point_cloud.point_step + point_cloud.fields[3].offset;

    *reinterpret_cast<float*>(&point_cloud.data[idx_point_x]) = std::numeric_limits<float>::quiet_NaN();
    *reinterpret_cast<float*>(&point_cloud.data[idx_point_y]) = std::numeric_limits<float>::quiet_NaN();
    *reinterpret_cast<float*>(&point_cloud.data[idx_point_z]) = std::numeric_limits<float>::quiet_NaN();
    *reinterpret_cast<float*>(&point_cloud.data[idx_sigma])   = std::numeric_limits<float>::quiet_NaN();    
  }
}

SensorTofHardware::Parameter
SensorTofHardware::get_parameter(const std::string& name, const Parameter& default_parameter, rclcpp::Node& ros_node)
{
  SensorTofHardware::Parameter parameter;

  // parameter declaration
  ros_node.declare_parameter<int>(name + ".sensor_id", default_parameter.sensor_id);

  ros_node.declare_parameter<int>(
    name + ".number_of_zones.vertical", default_parameter.number_of_zones.vertical);
  ros_node.declare_parameter<int>(
    name + ".number_of_zones.horizontal", default_parameter.number_of_zones.horizontal);
  ros_node.declare_parameter<float>(name + ".fov.vertical", default_parameter.fov.vertical);
  ros_node.declare_parameter<float>(name + ".fov.horizontal", default_parameter.fov.horizontal);
  ros_node.declare_parameter<int>(
    name + ".measurement_interval_ms", default_parameter.measurement_interval.count());

  // parameter reading
  parameter.sensor_id = ros_node.get_parameter(name + ".sensor_id").as_int();

  parameter.number_of_zones.vertical = ros_node.get_parameter(name + ".number_of_zones.vertical").as_int();
  parameter.number_of_zones.horizontal = ros_node.get_parameter(name + ".number_of_zones.horizontal").as_int();
  parameter.fov.vertical = ros_node.get_parameter(name + ".fov.vertical").as_double();
  parameter.fov.horizontal = ros_node.get_parameter(name + ".fov.horizontal").as_double();
  parameter.measurement_interval = std::chrono::milliseconds(
    ros_node.get_parameter(name + ".measurement_interval_ms").as_int());
  parameter.trigger_measurement = default_parameter.trigger_measurement;


  return parameter;
}

SensorTofHardware::SensorTofHardware(
  const std::string& name, const Parameter& parameter, rclcpp::Node& ros_node, std::shared_ptr<Executer> executer,
  std::shared_ptr<Communicator> communicator, std::shared_ptr<SensorVirtualRange> virtual_range_sensor)
  : SensorPointCloud::SensorInterface()
  , _parameter(parameter)
  , _ros_node(ros_node)
  , _communication_node(std::make_shared<CommunicatorNode>(executer, communicator))
  , _virtual_range_sensor(virtual_range_sensor)
{
  (void)name;

  _can_id.measurement = 0x308 + _parameter.sensor_id;

  auto rx_data_end_point = CanRxDataEndPoint::make_data_endpoint(
    executer,
    _can_id.measurement,
    std::bind(&SensorTofHardware::processRxData, this, std::placeholders::_1),
    8
  );
  _communication_node->registerRxDataEndPoint(rx_data_end_point);
}

// is called by the executer thread of rx data endpoint
void SensorTofHardware::processRxData(const message::RxMessageDataBuffer& data)
{
  
  if (MeasurementComplete::hasCorrectLength(data)) {
    // Measurement Complete
    if (MeasurementComplete::sensor(data) != _parameter.sensor_id){
      RCLCPP_ERROR(rclcpp::get_logger("SensorTofHardware"), "sensor ID doesn't match with sensor can address ");
    }

    // Notify sensor_tof_ring_hardware
    if(_callback_finished_measurement != nullptr){
      _callback_finished_measurement();
    }

    // TODO: If callback == 0, how should data transfer be triggered? 
    return;
  }
  
  if (DataTransmissionComplete::hasCorrectLength(data)) {
    // Data Transmission complete
    if (_callback_process_measurement == nullptr) {
      return;
    }

    if (_processing_data.point_counter != DataTransmissionComplete::pointCount(data)) {
      RCLCPP_ERROR(rclcpp::get_logger("SensorTofHardware"), "count of processed points is no equal to transmitted points");
    }

    // Calling Layer Above
    _callback_process_measurement(*_processing_data.point_cloud);
    if (_virtual_range_sensor) {
      _virtual_range_sensor->processPointCloudMeasurement(*_processing_data.point_cloud);
    }

    // Preparing Next Iteration
    _processing_data.next_expected_zone = 0;
    _processing_data.point_counter = 0;
    _processing_data.point_index = 0;
    set_nan_points(*_processing_data.point_cloud);

    return;
  }

  
  if(ZoneMeasurement::elements(data) == 16)
  {
    // receiving measurement data package (16 bytes) 
    for(std::size_t i = 0; i < ZoneMeasurement::elements(data); i++){

      const auto index = _processing_data.point_index;
      const auto distance = ZoneMeasurement::distance(data, i);
      const auto sigma = ZoneMeasurement::sigma(data, i);

      auto& point_cloud = _processing_data.point_cloud;
      const std::size_t idx_point_x = index * point_cloud->point_step + point_cloud->fields[0].offset;
      const std::size_t idx_point_y = index * point_cloud->point_step + point_cloud->fields[1].offset;
      const std::size_t idx_point_z = index * point_cloud->point_step + point_cloud->fields[2].offset;
      const std::size_t idx_sigma   = index * point_cloud->point_step + point_cloud->fields[3].offset;

      if (distance == 0.0f && sigma == 0.0f) {
        // empty measurement (signaled by 3 byte = 0) 
        *reinterpret_cast<float*>(&point_cloud->data[idx_point_x]) = std::numeric_limits<float>::quiet_NaN();
        *reinterpret_cast<float*>(&point_cloud->data[idx_point_y]) = std::numeric_limits<float>::quiet_NaN();
        *reinterpret_cast<float*>(&point_cloud->data[idx_point_z]) = std::numeric_limits<float>::quiet_NaN();
        *reinterpret_cast<float*>(&point_cloud->data[idx_sigma])   = std::numeric_limits<float>::quiet_NaN();
      }
      else{
        // valid measurement
        *reinterpret_cast<float*>(&point_cloud->data[idx_point_x]) = _processing_data.tan_x_lookup[index] * distance;
        *reinterpret_cast<float*>(&point_cloud->data[idx_point_y]) = _processing_data.tan_y_lookup[index] * distance;
        *reinterpret_cast<float*>(&point_cloud->data[idx_point_z]) = distance;
        *reinterpret_cast<float*>(&point_cloud->data[idx_sigma])   = sigma;
        _processing_data.point_counter++;
      }

      _processing_data.current_zone = index;            // TODO: still needed?
      _processing_data.next_expected_zone = index + 1;  // TODO: still needed?
      _processing_data.point_index++;
      // TODO: Check if point_index exceeds the expected number of points (expected number is set by parameter)
    }

    return;
  }

  // Error, received unexpected message
  RCLCPP_ERROR(rclcpp::get_logger("SensorTofHardware"), "received unexpected message");
}

void SensorTofHardware::initialize(const SensorPointCloud::Parameter& parameter)
{
  (void)parameter;

  // Preparing Processing Data
  _processing_data.number_of_zones = _parameter.number_of_zones.horizontal * _parameter.number_of_zones.vertical;
  _processing_data.current_zone = 0;
  _processing_data.next_expected_zone = 0;
  _processing_data.point_cloud = create_point_cloud(_parameter);
  _processing_data.frame_number = 0;  
  _processing_data.tan_x_lookup.resize(_processing_data.number_of_zones);
  _processing_data.tan_y_lookup.resize(_processing_data.number_of_zones);
  _processing_data.point_counter = 0;
  _processing_data.point_index = 0;
  set_nan_points(*_processing_data.point_cloud);

  const float alpha_increment_vertical = _parameter.fov.vertical / static_cast<double>(_parameter.number_of_zones.vertical);
  const float alpha_increment_horizontal = _parameter.fov.horizontal / static_cast<double>(_parameter.number_of_zones.horizontal);

  // \todo skip idx_beam == 0
  for (std::size_t idx_height = 0; idx_height < _parameter.number_of_zones.horizontal; ++idx_height) {
    const int idx_beam_y = ((static_cast<float>(_parameter.number_of_zones.vertical) / 2.0f) - idx_height);

    for (std::size_t idx_width = 0; idx_width < _parameter.number_of_zones.vertical; ++idx_width) {
      const std::size_t idx_zone = idx_height * _parameter.number_of_zones.horizontal + idx_width;
      const int idx_beam_x = ((static_cast<float>(_parameter.number_of_zones.horizontal) / 2.0f) - idx_width);

      _processing_data.tan_x_lookup[idx_zone] = std::tan(idx_beam_x * alpha_increment_horizontal);
      _processing_data.tan_y_lookup[idx_zone] = std::tan(idx_beam_y * alpha_increment_vertical);
    }
  }

  if (_parameter.trigger_measurement) {
    _communication_node->addSendingJob(
      std::bind(&SensorTofHardware::processMeasurement, this),
      _parameter.measurement_interval
    );
  }
}

// is called by the executer thread
void SensorTofHardware::processMeasurement()
{
  try {
    // Get measurement data from can gateway and parse it to processing pipeline.
    auto request = Request::make_request<StartMeasurement>(
      _can_id.trigger, _processing_data.frame_number, (1 << (_parameter.sensor_id - 1))
    );

    _communication_node->sendRequest(std::move(request), 100ms);

    _processing_data.point_cloud->header.stamp = _ros_node.get_clock()->now();
    _processing_data.frame_number++;
    _processing_data.current_zone = 0;
    _processing_data.next_expected_zone = 0;
  }
  catch (std::exception& ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SensorTofHardware"),
      "error occurred during processing measurement. what = %s.", ex.what()
    );
  }
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
