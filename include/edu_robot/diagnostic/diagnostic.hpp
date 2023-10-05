/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <map>
#include <string>
#include <sstream>

namespace eduart {
namespace robot {
namespace diagnostic {

class Diagnostic
{
public:
  enum class Level {
    OK = 1,
    WARN,
    ERROR, // needs to be the highest value
  };

  template <typename Value>
  void add(const std::string& key, const Value& value, const Level level)
  {
    // take key value pair
    std::stringstream ss;
    ss << value;
    _diagnostic_entry[key] = ss.str();

    // take level only if new level is higher
    if (level > _level) {
      _level = level;
    }
  }

  inline const std::map<std::string, std::string>& entries() const { return _diagnostic_entry; }
  inline Level level() const { return _level; }

private:
  Level _level = Level::OK;
  std::map<std::string, std::string> _diagnostic_entry;
};

template <>
inline void Diagnostic::add<std::string>(const std::string& key, const std::string& value, const Level level)
{
  // take key value pair
  _diagnostic_entry[key] = value;

  // take level only if new level is higher
  if (level > _level) {
    _level = level;
  }  
}

inline std::remove_const_t<decltype(diagnostic_msgs::msg::DiagnosticStatus::OK)> convert(
  const Diagnostic::Level level)
{
  switch (level) {
    case Diagnostic::Level::OK:
      return diagnostic_msgs::msg::DiagnosticStatus::OK;

    case Diagnostic::Level::WARN:
      return diagnostic_msgs::msg::DiagnosticStatus::WARN;

    case Diagnostic::Level::ERROR:
      return diagnostic_msgs::msg::DiagnosticStatus::ERROR;

    default:
      return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
}


} // end namespace diagnostic
} // end namespace robot
} // end namespace eduart
