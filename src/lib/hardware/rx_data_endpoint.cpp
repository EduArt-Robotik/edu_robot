#include "edu_robot/hardware/rx_data_endpoint.hpp"
#include "edu_robot/hardware/communicator_device_interfaces.hpp"

#include <iostream>

namespace eduart {
namespace robot {
namespace hardware {

using namespace std::chrono_literals;

RxDataEndPoint::RxDataEndPoint(
  std::vector<message::Byte>& search_pattern,
  const std::function<void(const message::RxMessageDataBuffer&)>& callback_process_data,
  CommunicatorRxDevice* data_receiver)
  : _response_search_pattern(std::move(search_pattern))
  , _callback_process_data(callback_process_data)
  , _data_receiver(data_receiver)
{
  _data_buffer.reserve(100);
  _executer = std::thread(&RxDataEndPoint::processDataJob, this);
}

void RxDataEndPoint::deactivate()
{
  _running = false;
}

void RxDataEndPoint::call(const message::RxMessageDataBuffer& data)
{
  if (_running == false) {
    // Data endpoint not active --> do nothing
    return;
  }
  if (_mutex.try_lock() == false) {
    RCLCPP_ERROR(rclcpp::get_logger("RxDataEndPoint"), "data receiver is still busy. Drop data.");

    std::stringstream debug_out;
    debug_out << "data: " << std::hex;

    for (const auto byte : data) {
      debug_out << static_cast<int>(byte) << " ";
    }
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("RxDataEndPoint"), debug_out.str());
    return;
  }

  std::cout << "capacity before copy = " << _data_buffer.capacity() << std::endl;
  _data_buffer = data;
  std::cout << "capacity after copy = " << _data_buffer.capacity() << std::endl;
  _mutex.unlock();  
}

void RxDataEndPoint::processDataJob()
{
  while (_running) {
    if (_data_receiver == nullptr) {
      RCLCPP_ERROR(rclcpp::get_logger("RxDataEndPoint"), "data receiver does not longer exist.");
    }

    _mutex.lock();

    if (_data_buffer.empty()) {
      // no data to process --> sleep and try again
      _mutex.unlock();
      std::this_thread::sleep_for(1ms);
      continue;
    }

    std::scoped_lock lock_receiver(_data_receiver->rxDataMutex());
    _callback_process_data(_data_buffer);
    _data_buffer.clear();
    std::cout << "capacity after clear = " << _data_buffer.capacity() << std::endl;
    _mutex.unlock();
  }
}

} // end namespace igus
} // end namespace eduart
} // end namespace robot
