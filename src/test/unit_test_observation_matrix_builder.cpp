#include <catch2/catch.hpp>

#include <edu_robot/algorithm/observation_matrix_builder.hpp>
#include <edu_robot/algorithm/kinematic_state_vector.hpp>

#include <iostream>

using namespace eduart::robot::algorithm;
using namespace eduart::robot::algorithm::kinematic;

TEST_CASE("observation_matrix_builder", "[unittest]")
{
  using StateAttributes = AttributePack<Attribute::POS_X, Attribute::POS_Y, Attribute::VEL_X, Attribute::VEL_Y, Attribute::YAW, Attribute::YAW_RATE>;
  using SensorAttributes = AttributePack<Attribute::VEL_X, Attribute::VEL_Y>;
  using FilterStateVector = StateVector<StateAttributes>;
  using MeasurementVector = StateVector<SensorAttributes>;

  const auto observation_matrix = ObservationMatrixBuilder<StateAttributes, SensorAttributes>::build();

  SECTION("Check Processed Matrix Entries") {
    REQUIRE(observation_matrix.rows() == StateAttributes::count());
    REQUIRE(observation_matrix.cols() == SensorAttributes::count());

    // Check State Entries against Sensor VEL_X attribute.
    CHECK(observation_matrix(0, 0) == 0);
    CHECK(observation_matrix(0, 0) == 0);
    CHECK(observation_matrix(0, 0) == 0);
    CHECK(observation_matrix(0, 0) == 0);
    CHECK(observation_matrix(0, 0) == 0);
    CHECK(observation_matrix(0, 0) == 0);

    std::cout << "observation_matrix:\n" << observation_matrix << std::endl;
  }
}
