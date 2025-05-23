#include "edu_robot/hardware/iot_shield/uart/message_definition.hpp"
#include "edu_robot/hardware/rx_data_endpoint.hpp"

#include <edu_robot/hardware/iot_shield/imu_sensor_hardware.hpp>
#include <edu_robot/hardware/communicator_node.hpp>
#include <edu_robot/hardware/iot_shield/uart/uart_request.hpp>

#include <edu_robot/sensor_imu.hpp>

#include <Eigen/src/Core/Matrix.h>
#include <functional>
#include <memory>

namespace eduart {
namespace robot {
namespace hardware {
namespace iot_shield {

using namespace std::chrono_literals;

using hardware::iot_shield::uart::Request;
using hardware::iot_shield::uart::message::ShieldResponse;
using hardware::iot_shield::uart::message::SetImuRawDataMode;

ImuSensorHardware::ImuSensorHardware(std::shared_ptr<Executer> executer, std::shared_ptr<Communicator> communicator)
  : _communication_node(std::make_shared<CommunicatorNode>(executer, communicator))
{
  _communication_node->createRxDataEndPoint<RxDataEndPoint, ShieldResponse>(
    std::bind(&ImuSensorHardware::processRxData, this, std::placeholders::_1), 3
  );
}

void ImuSensorHardware::processRxData(const message::RxMessageDataBuffer& data)
{
  if (_callback_process_measurement == nullptr) {
    return;
  }
  
  if (_raw_mode == false) {
    _callback_process_measurement(
      ShieldResponse::imuOrientation(data), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()
    );
  }
  else {
    _callback_process_measurement(
      Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
      ShieldResponse::angularVelocity(data),
      ShieldResponse::linearAcceleration(data)
    );
  }
}

void ImuSensorHardware::initialize(const SensorImu::Parameter& parameter)
{
  // set IMU data mode
  auto request = Request::make_request<SetImuRawDataMode>(parameter.raw_data_mode, 0, 0, 0);
  _communication_node->sendRequest(std::move(request), 100ms);

  _raw_mode = parameter.raw_data_mode;
}

} // end namespace iot_shield
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
