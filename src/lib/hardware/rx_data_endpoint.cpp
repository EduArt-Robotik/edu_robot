#include "edu_robot/hardware/rx_data_endpoint.hpp"
#include "edu_robot/hardware/communicator_node.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <cstdint>
#include <mutex>

namespace eduart {
namespace robot {
namespace hardware {

using namespace std::chrono_literals;

RxDataEndPoint::RxDataEndPoint(
  std::vector<message::Byte>& search_pattern,
  const std::function<void(const message::RxMessageDataBuffer&)>& callback_process_data,
  std::shared_ptr<Executer> executer,
  const std::uint8_t buffer_size)
  : _input_data_buffer(buffer_size)
  , _executer(executer)
  , _response_search_pattern(std::move(search_pattern))
  , _callback_process_data(callback_process_data)
{
  // Preallocate memory.
  for (auto& buffer : _input_data_buffer) {
    buffer = std::make_shared<message::RxMessageDataBuffer>();
    buffer->reserve(100);
  }

  // Register at executer.
  _executer->addJob(std::bind(&RxDataEndPoint::processDataJob, this), 1ms);
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
  *buffer = data;

  // Add data to output queue.
  _mutex.lock();
  _output_data_buffer.push(buffer);
  _mutex.unlock();
}

void RxDataEndPoint::processDataJob()
{
  _mutex.lock();

  if (_output_data_buffer.empty()) {
    // no data to process --> nothing to do --> return
    _mutex.unlock();
    return;
  }

  // Take data buffer from queue.
  auto buffer = _output_data_buffer.front();
  _output_data_buffer.pop();
  _mutex.unlock();

  // Process data buffer.
  _callback_process_data(*buffer);
  buffer->clear();

  // Move back empty data buffer to input buffer. Buffer keeps allocated memory.
  _mutex.lock();
  _input_data_buffer.push_back(buffer);
  _mutex.unlock();
}

} // end namespace igus
} // end namespace eduart
} // end namespace robot
