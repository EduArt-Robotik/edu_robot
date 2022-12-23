#include "edu_robot/algorithm/kinematic_attribute.hpp"
#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>

#include <edu_robot/algorithm/kinematic_state_vector.hpp>

using namespace eduart::robot::algorithm::kinematic;

TEST_CASE("kinematic_state_vector", "[unittest]")
{
  StateVector<AttributePack<Attribute::POS_X, Attribute::POS_Y>> vector;

  vector.get<Attribute::POS_X>();
}
