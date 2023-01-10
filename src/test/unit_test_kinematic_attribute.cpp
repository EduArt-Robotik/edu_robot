#include <catch2/catch.hpp>
#include <cstddef>
#include <iostream>

#include "edu_robot/algorithm/kinematic_attribute.hpp"

using namespace eduart::robot::algorithm::kinematic;

TEST_CASE("kinematic_attribute_pack", "[unittest]")
{
  using Pack = AttributePack<Attribute::POS_X, Attribute::POS_Y, Attribute::YAW, Attribute::YAW_RATE>;
  constexpr std::size_t index = Pack::index<Attribute::POS_X>();
  std::cout << index << std::endl;
  SECTION("method index") {
    CHECK(Pack::index<Attribute::POS_X>() == 0);
    CHECK(Pack::index<Attribute::POS_Y>() == 1);
    CHECK(Pack::index<Attribute::YAW>() == 2);
    CHECK(Pack::index<Attribute::YAW_RATE>() == 3);
  }
  SECTION("method attribute") {
    CHECK(Pack::attribute<0>() == Attribute::POS_X);
    CHECK(Pack::attribute<1>() == Attribute::POS_Y);
    CHECK(Pack::attribute<2>() == Attribute::YAW);
    CHECK(Pack::attribute<3>() == Attribute::YAW_RATE);            
  }
  SECTION("method isContained") {
    CHECK_FALSE(Pack::isContained<Attribute::ACC_X>());
    CHECK_FALSE(Pack::isContained<Attribute::ACC_Y>());
    CHECK_FALSE(Pack::isContained<Attribute::ACC>());
    CHECK_FALSE(Pack::isContained<Attribute::VEL>());
    CHECK_FALSE(Pack::isContained<Attribute::VEL_X>());
    CHECK_FALSE(Pack::isContained<Attribute::VEL_Y>());
    CHECK_FALSE(Pack::isContained<Attribute::ROLL>());
    CHECK_FALSE(Pack::isContained<Attribute::PITCH>());
    CHECK_FALSE(Pack::isContained<Attribute::ROLL_RATE>());
    CHECK_FALSE(Pack::isContained<Attribute::PITCH_RATE>());

    CHECK(Pack::isContained<Attribute::POS_X>());
    CHECK(Pack::isContained<Attribute::POS_Y>());
    CHECK(Pack::isContained<Attribute::YAW>());
    CHECK(Pack::isContained<Attribute::YAW_RATE>());
  }
  SECTION("count") {
    CHECK(Pack::count() == 4);
  }
}
