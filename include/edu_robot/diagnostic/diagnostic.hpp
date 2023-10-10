/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/diagnostic/diagnostic_level.hpp"
#include "edu_robot/diagnostic/diagnostic_check.hpp"

#include <map>
#include <string>
#include <sstream>

namespace eduart {
namespace robot {
namespace diagnostic {

class Diagnostic
{
public:
  template <typename Value>
  void add(const std::string& key, const Value& value, const Level level)
  {
    // take key value pair
    std::stringstream ss;
    ss << value;
    _diagnostic_entry[key] = ss.str();
    _diagnostic_level[key] = level;

    // take level only if new level is higher
    if (level > _level) {
      _level = level;
    }
  }
  
  void add(const DiagnosticCheckList& check)
  {
    check.addToLevelList(_diagnostic_level);
    check.addToEntryList(_diagnostic_entry);
  }

  inline const DiagnosticCheckList::EntryList& entries() const { return _diagnostic_entry; }
  inline Level level() const { return _level; }
  /**
   * \brief Generates an diagnostic summery string. All warning and errors will be included into this stirng.
   */
  std::string summery() const
  {
    std::string result;
    std::string error_string;
    std::string warning_string;

    const auto count_errors = std::count_if(
      _diagnostic_level.begin(), _diagnostic_level.end(), [this, &error_string](const auto entry){
        if (entry.second == Level::ERROR) {
          error_string += entry.first + ' ' + _diagnostic_entry.at(entry.first) + ',';
          return true;
        }
        // else
        return false;
      }
    );
    const auto count_warnings = std::count_if(
      _diagnostic_level.begin(), _diagnostic_level.end(), [this, &warning_string](const auto entry){
        if (entry.second == Level::WARN) {
          warning_string += entry.first + ' ' + _diagnostic_entry.at(entry.first) + ',';
          return true;
        }
        // else
        return false;
      }
    );

    // case: no error or warning is occurred
    if (count_errors == 0 && count_warnings == 0) {
      return "working properly";
    }

    // case: error occurred
    if (count_errors == 1) {
      result += "1 error ";
    }
    else if (count_errors > 1) {
      result += std::to_string(count_errors) + " errors ";
    }
    if (count_errors > 0) {
      result += '(';
      result += error_string;
      result.back() = ')';
      result += ' ';
    }

    // case: warning occurred
    if (count_warnings == 1) {
      result += "1 warning ";
    }
    else if (count_warnings > 1) {
      result += std::to_string(count_warnings) + " warnings ";
    }
    if (count_warnings > 0) {
      result += '(';
      result += warning_string;
      result.back() = ')';
    }

    return result;
  }

private:
  Level _level = Level::OK;
  DiagnosticCheckList::EntryList _diagnostic_entry;
  DiagnosticCheckList::LevelList _diagnostic_level;
};

template <>
inline void Diagnostic::add<std::string>(const std::string& key, const std::string& value, const Level level)
{
  // take key value pair
  _diagnostic_entry[key] = value;
  _diagnostic_level[key] = level;

  // take level only if new level is higher
  if (level > _level) {
    _level = level;
  }
}

} // end namespace diagnostic
} // end namespace robot
} // end namespace eduart
