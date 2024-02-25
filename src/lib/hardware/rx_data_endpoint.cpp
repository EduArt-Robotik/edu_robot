#include "edu_robot/hardware/rx_data_endpoint.hpp"
#include "edu_robot/hardware/communicator_device_interfaces.hpp"

#include <cstdint>
#include <iostream>
#include <mutex>

namespace eduart {
namespace robot {
namespace hardware {

using namespace std::chrono_literals;

RxDataEndPoint::RxDataEndPoint(
  std::vector<message::Byte>& search_pattern,
  const std::function<void(const message::RxMessageDataBuffer&)>& callback_process_data,
  CommunicatorRxDevice* data_receiver,
  const std::uint8_t buffer_size)
  : _input_data_buffer(buffer_size)
  , _response_search_pattern(std::move(search_pattern))
  , _callback_process_data(callback_process_data)
  , _data_receiver(data_receiver)
{
  // Preallocate memory.
  for (auto& buffer : _input_data_buffer) {
    buffer = std::make_shared<message::RxMessageDataBuffer>();
    buffer->reserve(100);
  }

  // Start executer.
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

  // Get next index and check if there is room for new input data. Input index must stay in front of output index.
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  _mutex.lock();
  std::cout << "input buffer size = " << static_cast<int>(_input_data_buffer.size()) << std::endl;

  if (_input_data_buffer.empty()) {
    // No data slot left --> buffer is full --> cancel
    _mutex.unlock();
    RCLCPP_ERROR(rclcpp::get_logger("RxDataEndPoint"), "data receiver is still busy. Drop data.");

    std::stringstream debug_out;
    debug_out << "data: " << std::hex;

    for (const auto byte : data) {
      debug_out << static_cast<int>(byte) << " ";
    }
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("RxDataEndPoint"), debug_out.str());
    return;
  }
  // else:
  // Room for new input data --> copy data

  // Get available buffer and copy input data to it.
  auto buffer = _input_data_buffer.back();
  _input_data_buffer.pop_back();
  _mutex.unlock();
  std::cout << "copy data..." << std::endl;
  *buffer = data;

  // Add data to output queue.
  _mutex.lock();
  _output_data_buffer.push(buffer);
  _mutex.unlock();
}

void RxDataEndPoint::processDataJob()
{
  while (_running) {
    _mutex.lock();

    if (_output_data_buffer.empty()) {
      // no data to process --> sleep and try again
      _mutex.unlock();
      std::this_thread::sleep_for(1ms);
      continue;
    }

    // Take data buffer from queue.
    auto buffer = _output_data_buffer.front();
    _output_data_buffer.pop();
    std::cout << "output buffer size = " << static_cast<int>(_output_data_buffer.size()) << std::endl;
    _mutex.unlock();

    // Process data buffer.
    std::scoped_lock lock_receiver(_data_receiver->rxDataMutex());
    _callback_process_data(*buffer);
    buffer->clear();

    // Move back empty data buffer to input buffer.
    _mutex.lock();
    _input_data_buffer.push_back(buffer);
    _mutex.unlock();
  }
}

} // end namespace igus
} // end namespace eduart
} // end namespace robot
