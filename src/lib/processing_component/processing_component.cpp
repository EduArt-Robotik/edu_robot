#include "edu_robot/processing_component/processing_component.hpp"

namespace eduart {
namespace robot {
namespace processing {

PortInput::PortInput(const PortInput& lhs)
  : Port(lhs)
  , _max_queue_size(lhs._max_queue_size)
  , _input_queue(lhs._input_queue)
  , _mutex()
{

}

void PortInput::addValue(const Data& data)
{
  std::scoped_lock lock(_mutex);

  if (_input_queue.size() >= _max_queue_size) {
    throw std::runtime_error("Max queue size reached. Can't add new value to input.");
  }

  _input_queue.push(data);
}

Data PortInput::getValue()
{
  std::scoped_lock lock(_mutex);

  Data data = _input_queue.front();
  _input_queue.pop();

  return data;
}


void PortOutput::connect(std::shared_ptr<PortInput> input)
{
  if (input->type() != this->type()) {
    throw std::invalid_argument("Given port input type does not match output type.");
  }

  _inputs.push_back(input);
}

void PortOutput::disconnect(std::shared_ptr<PortInput> input)
{
  const auto search = std::find(_inputs.begin(), _inputs.end(), input);

  if (search == _inputs.end()) {
    throw std::invalid_argument("Given port input is not connected to this output.");
  }

  _inputs.erase(search);
}


PortInput& ProcessingComponent::input(const std::string& name)
{
  return _inputs.at(name);
}

PortOutput& DataSourceComponent::output(const std::string& name)
{
  return _outputs.at(name);
}


} // end namespace processing
} // end namespace robot
} // end namespace eduart
