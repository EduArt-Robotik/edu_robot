#define CATCH_CONFIG_MAIN

#if __has_include(<catch2/catch_all.hpp>)
#include <catch2/catch_all.hpp>
#else
#include <catch2/catch.hpp>
#endif

#include <edu_robot/hardware/can_gateway/can_communication_device.hpp>
#include <edu_robot/hardware/can_gateway/motor_controller_hardware.hpp>

// Tests the communication with can motor controller first. After some features are tested that can be tested via CAN interface.
TEST_CASE("can communication", "[MotorControllerHardware]")
{
  using namespace std::chrono_literals;
  using eduart::robot::hardware::can_gateway::MotorControllerHardware;
  using eduart::robot::hardware::can_gateway::CanCommunicationDevice;
  using eduart::robot::hardware::Communicator;
  using eduart::robot::Executer;
  using eduart::robot::Motor;

  const MotorControllerHardware::Parameter parameter_hardware = {
    {0x400, 0x480},
    16000,
    666ms,
    0.45f
  };
  auto can_device = std::make_shared<CanCommunicationDevice>("eduart-can2", CanCommunicationDevice::CanType::CAN);
  auto communicator = std::make_shared<Communicator>(can_device, 1ms);
  auto executer = std::make_shared<Executer>(1ms);
  MotorControllerHardware controller("can_motor_controller", parameter_hardware, executer, communicator);

  const Motor::Parameter parameter_motor_a = {
    true,
    false,
    0,
    666.0,
    47.11,
    {
      1.0,
      2.0,
      3.0,
    },
    {
      false,
      1313
    }
  };
  const Motor::Parameter parameter_motor_b = {
    false,
    true,
    0,
    47.11,
    666.0,
    {
      2.0,
      3.0,
      4.0,
    },
    {
      false,
      2323
    }
  };

  SECTION("parameter_combination_one") {
    // initial motors with 
    std::vector<Motor::Parameter> parameter_motor_set = { parameter_motor_a, parameter_motor_b };

    controller.initialize(parameter_motor_set);
  }

  SECTION("parameter_combination_two") {
    // initial motors with 
    std::vector<Motor::Parameter> parameter_motor_set = { parameter_motor_b, parameter_motor_a };

    controller.initialize(parameter_motor_set);
  }
}
