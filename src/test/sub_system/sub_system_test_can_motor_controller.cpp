#define CATCH_CONFIG_MAIN

#include <catch2/catch_all.hpp>

#include <edu_robot/hardware/can_gateway/motor_controller_hardware.hpp>

// Tests the communication with can motor controller first. After some features are tested that can be tested via CAN interface.
TEST_CASE("can communication", "[MotorControllerHardware]")
{
  using namespace std::chrono_literals;

  const eduart::robot::hardware::can_gateway::MotorControllerHardware::Parameter parameter_hardware = {
    {0x400, 0x480},
    16000,
    666ms,
    0.45f
  };
  const eduart::robot::Motor::Parameter parameter_motor = {
    
  }
}