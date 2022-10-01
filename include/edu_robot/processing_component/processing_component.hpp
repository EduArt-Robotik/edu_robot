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

namespace eduart {
namespace robot {
namespace processing {

template <typename Output>
class ProcessingComponentOutput;

template <typename Input>
class ProcessingComponentInput
{
public:
  virtual ~ProcessingComponentInput() = default;

  virtual void processInput(const Input& value, const ProcessingComponentOutput<Input>* sender) = 0;
};

template <typename Output>
class ProcessingComponentOutput
{
public:
  virtual ~ProcessingComponentOutput() = default;

  void registerComponentInput(std::shared_ptr<ProcessingComponentInput<Output>> input)
  {
    if (std::find(_inputs.begin(), _inputs.end(), input) != _inputs.end()) {
      throw std::invalid_argument("Given component input is already registered. Can't add it twice.");      
    }

    _inputs.push_back(input);
  }

protected:
  void sendInputValue(const Output& value)
  {
    for (auto& input : _inputs) {
      input->processInput(value, this);
    }
  }

private:
  std::vector<std::shared_ptr<ProcessingComponentInput<Output>>> _inputs;
};

class ProcessingComponent
{
protected:
  ProcessingComponent(const std::string& name, rclcpp::Node& ros_node)
    : _name(name)
    , _clock(ros_node.get_clock())
    , _last_processing(_clock->now())
  { }

public:
  ProcessingComponent(const ProcessingComponent&) = delete;
  virtual ~ProcessingComponent() = default;

private:
  std::string _name;
  std::shared_ptr<rclcpp::Clock> _clock;

protected:
  rclcpp::Time _last_processing;
};

} // end namespace processing
} // end namespace robot
} // end namespace eduart
