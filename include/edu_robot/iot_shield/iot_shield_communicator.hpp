/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/iot_shield/uart/message.hpp"

#if _WITH_MRAA
#include <mraa/common.hpp>
#include <mraa/uart.hpp>
#endif

#include <functional>
#include <memory>
#include <array>
#include <chrono>

namespace eduart {
namespace robot {
namespace iotbot {

class IotShieldDevice;



class IotShieldCommunicator
{
public:
  IotShieldCommunicator(char const* const device_name);

  void registerDevice(std::shared_ptr<IotShieldDevice> device);
  void registerProcessReceivedBytes(std::function<void(const uart::message::RxMessageDataBuffer&)> callback);
  void sendBytes(const uart::message::TxMessageDataBuffer& bytes);

private:
  void receiveBytes();

  std::function<void(const uart::message::RxMessageDataBuffer&)> _process_received_bytes;

#if _WITH_MRAA
  std::unique_ptr<mraa::Uart> _uart;
#endif
  std::chrono::milliseconds _wait_time_after_sending;
  std::chrono::time_point<std::chrono::system_clock> _last_sending;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
