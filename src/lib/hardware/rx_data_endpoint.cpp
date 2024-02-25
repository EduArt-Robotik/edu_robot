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
  : _data_buffer(buffer_size)
  , _response_search_pattern(std::move(search_pattern))
  , _callback_process_data(callback_process_data)
  , _data_receiver(data_receiver)
{
  // Preallocate memory.
  for (auto& buffer : _data_buffer) {
    buffer.reserve(100);
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
  _mutex.lock();
  const std::uint8_t next_index = _index_input + 1 >= _data_buffer.size() ? 0 : _index_input + 1;
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  std::cout << "next index = " << next_index << std::endl;

  if (next_index == _index_output) {
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

  // Copy input data to next index
  std::cout << "copy data..." << std::endl;
  _data_buffer[next_index] = data;

  // Take over new index --> new input data we be active/valid.
  _mutex.lock();
  _index_input = next_index;
  _mutex.unlock();
}

void RxDataEndPoint::processDataJob()
{
  while (_running) {
    _mutex.lock();

    if (_index_input == _index_output && _data_buffer[_index_output].empty()) {
      // no data to process --> sleep and try again
      _mutex.unlock();
      std::this_thread::sleep_for(1ms);
      continue;
    }

    std::cout << "process output index = " << _index_output << std::endl;
    _mutex.unlock();
    std::scoped_lock lock_receiver(_data_receiver->rxDataMutex());
    _callback_process_data(_data_buffer[_index_output]);
    _data_buffer[_index_output].clear();

    _mutex.lock();
    if (_index_input != _index_output) {
      // still work to do --> iterate to next index
      _index_output = _index_output + 1 >= _data_buffer.size() ? 0 : _index_output + 1;
      std::cout << "new output index = " << _index_output << std::endl;
    }
    // else:
    // nothing to do --> stay on index, output index must not overtake input index
    std::cout << "stay on output index = " << _index_output << std::endl;
    _mutex.unlock();
  }
}

} // end namespace igus
} // end namespace eduart
} // end namespace robot
