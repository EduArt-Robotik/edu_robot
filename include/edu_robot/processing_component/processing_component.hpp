/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>
#include <queue>

namespace eduart {
namespace robot {
namespace processing {

class Port
{
protected:
  Port(const std::string& name, const std::type_index type) : _name(name), _type(type) { }

public:
  Port(const Port&) = default;
  // Port(Port&&) = default;
  virtual ~Port() = default;  

  const std::string& name() const { return _name; }
  std::type_index type() const { return _type; }

private:
  std::string _name;
  std::type_index _type;
};

class Data
{
public:
  Data() : _data(nullptr), _type(typeid(void)) { };
  template <typename DataType>
  Data(const DataType& data) : _type(std::type_index(typeid(DataType))) { set<DataType>(data); }

  template <typename DataType>
  DataType get() const {
    if (_type != std::type_index(typeid(DataType))) {
      throw std::bad_cast();
    }
    return *std::static_pointer_cast<DataType>(_data);
 }
 template <typename DataType>
  void set(const DataType& data) {
    _data = std::make_shared<DataType>(data);
    _type = std::type_index(typeid(DataType));
  }

private:
  std::shared_ptr<void> _data;
  std::type_index _type;
};

/**
 * \brief Is an input for receiving data for processing components. The input has an queue that holds all sent input values.
 */
class PortOutput;

class PortInput : public Port
{
friend class PortOutput;

public:
  PortInput(const PortInput& lhs);
  virtual ~PortInput() = default;
  template <typename DataType>
  static PortInput make(const std::string& name, const std::size_t max_queue_size = 10) {
    return PortInput(name, max_queue_size, DataType());
  }

  Data getValue();
  inline bool hasValue() const {
    std::scoped_lock lock(_mutex);
    return !_input_queue.empty();
  }

private:
  template <typename DataType>
  explicit PortInput(const std::string& name, const std::size_t max_queue_size, const DataType& data_type)
    : Port(name, std::type_index(typeid(DataType)))
    , _max_queue_size(max_queue_size)
  { (void)data_type; }
  void addValue(const Data& data);

  std::size_t _max_queue_size = 10;
  std::queue<Data> _input_queue;
  mutable std::mutex _mutex;
};

class PortOutput : public Port
{
public:
  PortOutput(const PortOutput&) = default;
  virtual ~PortOutput() = default;

  void connect(std::shared_ptr<PortInput> input);
  void disconnect(std::shared_ptr<PortInput> input);

  template <typename DataType>
  void setValue(const DataType& data)
  {
    if (std::type_index(typeid(DataType)) != type()) {
      throw std::invalid_argument("Given data has wrong type!");
    }

    for (const auto& input : _inputs) {
      input->addValue(Data(data));
    }
  }

  template <typename DataType>
  static PortOutput make(const std::string& name) {
    return PortOutput(name, DataType());
  }

private:
  template <typename DataType>
  explicit PortOutput(const std::string& name, const DataType& data_type)
    : Port(name, std::type_index(typeid(DataType)))
  { (void)data_type; }

  std::string _name;
  std::vector<std::shared_ptr<PortInput>> _inputs;
};

class DataSourceComponent
{
public:
  // DataSourceComponent(const DataSourceComponent&) = default;
  virtual ~DataSourceComponent() = default;

  PortOutput& output(const std::string& name);

protected:
  template <typename DataType>
  void createOutput(const std::string& name) {
    _outputs.emplace(name, std::move(PortOutput::make<DataType>(name)));
  }

private:
  std::unordered_map<std::string, PortOutput> _outputs;
};

class ProcessingComponent : public DataSourceComponent
{
protected:
  ProcessingComponent(const std::string& name, rclcpp::Node& ros_node)
    : _name(name)
    , _clock(ros_node.get_clock())
    , _last_processing(_clock->now())
  { }

public:
  ProcessingComponent(const ProcessingComponent&) = delete;
  ~ProcessingComponent() override = default;

  const std::string& name() const { return _name; }
  virtual void process() = 0;

private:
  std::string _name;
  std::unordered_map<std::string, PortInput> _inputs;

protected:
  PortInput& input(const std::string& name);

  template <typename DataType>
  void createInput(const std::string& name, const std::size_t queue_size = 10) {
    auto port = PortInput::make<DataType>(name, queue_size);
    _inputs.emplace(name, std::move(port));
  }

  std::shared_ptr<rclcpp::Clock> _clock;
  rclcpp::Time _last_processing;
};

} // end namespace processing
} // end namespace robot
} // end namespace eduart
