/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/diagnostic/diagnostic_level.hpp"

#include <string>
#include <vector>
#include <unordered_map>

namespace eduart {
namespace robot {
namespace diagnostic {

/**
 * This class represent an diagnostic check that contains a warning and error level. It is a helper class to avoid code
 * duplications in diagnostic processing functions/methods. It only works on scalar values at the moment.
 */
template <typename Type, class Checker>
class DiagnosticCheck
{
public:
  DiagnosticCheck(const std::string& name, const Type level_warning, const Type level_error)
  : _name(name)
  , _level_warning(level_warning)
  , _level_error(level_error)
  { }

  inline Level estimateLevel(const Type value) const
  {
    if (Checker()(value, _level_error)) {
      return Level::ERROR;
    }
    else if (Checker()(value, _level_warning)) {
      return Level::WARN;
    }
    // else
      
    return Level::OK;
  }

  inline const std::string& name() const { return _name; }
  inline Type levelWarning() const { return _level_warning; }
  inline Type levelError() const { return _level_error; }

private:
  std::string _name;
  Type _level_warning;
  Type _level_error;
};

/**
 * \brief 
 */
class DiagnosticCheckList
{
public:
  using LevelList = std::unordered_map<std::string, Level>;
  using EntryList = std::unordered_map<std::string, std::string>;

  virtual ~DiagnosticCheckList() = default;

  virtual void addToLevelList(LevelList& list) const = 0;
  virtual void addToEntryList(EntryList& list) const = 0;

  LevelList levels() const {
    LevelList list;
    addToLevelList(list);
    return list;
  }
  EntryList entries() const {
    EntryList list;
    addToEntryList(list);
    return list;
  }
};

} // end namespace diagnostic
} // end namespace robot
} // end namespace eduart
