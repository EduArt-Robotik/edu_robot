/**
 * This file defines attributes for kinematic filters, also container that can handle such attributes in a conformable way.
 * Origin idea is from https://github.com/franc0r/libfrancor/blob/feature/kalman-filter/francor_mapping/include/francor_mapping/kinematic_attributes.h
 *
 * \author Christian Wendt (knueppl@gmx.de)
 * \date 23. December 2022
 */
#pragma once

#include <array>
#include <cassert>
#include <cstddef>
#include <tuple>

namespace eduart {
namespace robot {
namespace algorithm {
namespace kinematic {

enum class Attribute {
  POS_X,
  POS_Y,
  VEL,
  VEL_X,
  VEL_Y,
  ACC,
  ACC_X,
  ACC_Y,
  ROLL,
  PITCH,
  YAW,
  ROLL_RATE,
  PITCH_RATE,
  YAW_RATE,
};

/**
 * \brief This template struct can hold a set of kinematic Attributes. It provides methods to check if an attributes is
 *        contained or not and to get the index of an attribute.
 */
template <Attribute... Attributes>
struct AttributePack
{
  /**
   * \brief Returns the index of an attribute. If the attribute is not contained assert will fail.
   * 
   * \tparam Target The index of this attribute will be returned.
   * \return Index of the given attribute.
   */
  template <Attribute Target>
  inline static constexpr std::size_t index() {
    std::size_t counter = 0;
    bool found = false;

    ((found == false && Target == Attributes ? found = true : ++counter), ...);

    assert(found == true); // "Given Attribute is not contained in this AttributePack!");

    return counter;
  }
  /**
   * \brief Returns attribute of given index. Compilation fails if given index doesn't exist.
   * \tparam Index Index of wanted attribute.
   * \return Attribute of given index.
   */
  template <std::size_t Index>
  inline static constexpr Attribute attribute() {
    static_assert(Index < sizeof...(Attributes), "Given Index is out of range!");

    std::array<Attribute, sizeof...(Attributes)> helper = { Attributes... };
    return helper[Index];
  }

  template <Attribute Target>
  inline static constexpr bool isContained() {
    return countQuantityOfAnAttribute<Target>() != 0;
  }

  inline static constexpr std::size_t count() {
    return sizeof...(Attributes);
  }

private:
  /**
   * \brief Counts the quantity of given attribute in given Attribute parameter pack
   * \tparam Target Counts quantity of this attribute.
   * \return Quantity of given attribute.
   */
  template <Attribute Target>
  inline static constexpr std::size_t countQuantityOfAnAttribute() {
    std::size_t count = 0;

    ((Target == Attributes ? ++count : count), ...);

    return count;
  }

  // Checks if the attribute of this class exists only once. It starts at index 0 and works like:
  inline static constexpr bool checkConsistency() {
    bool okay = true;

    ((okay &= (countQuantityOfAnAttribute<Attributes>() == 1 ? true : false)), ...);
    return okay;
  }
  static_assert(checkConsistency() == true, "Each attribute must be exist only one time.");
};

} // end namespace kinematic
} // end namespace algorithm
} // end namespace robot
} // end namespace eduart
