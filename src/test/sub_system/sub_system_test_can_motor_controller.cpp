#define CATCH_CONFIG_MAIN

#if __has_include(<catch2/catch_all.hpp>)
#include <catch2/catch_all.hpp>
#else
#include <catch2/catch.hpp>
#endif

#include <rclcpp/rclcpp.hpp>

#include <edu_robot/hardware/can_gateway/can_communication_device.hpp>
#include <edu_robot/hardware/can_gateway/motor_controller_hardware.hpp>

using namespace std::chrono_literals;
using eduart::robot::hardware::can_gateway::MotorControllerHardware;
using eduart::robot::hardware::can_gateway::CanCommunicationDevice;
using eduart::robot::hardware::Communicator;
using eduart::robot::Executer;
using eduart::robot::Motor;

// Tests parameter setting and getting
TEST_CASE("parameter_handling", "[MotorControllerHardware]")
{
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

  // start testing
  executer->start();

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

// Tests if the motor controller disables after a timeout is occurred.
TEST_CASE("timeout", "[MotorControllerHardware]")
{
  using namespace std::chrono;

  class MotorState
  {
  public:
    inline bool isEnabled() const { return _is_enable; }
    void callbackFeedback(const std::vector<eduart::robot::Rpm>& rpm, const bool is_enabled) {
      (void)rpm;
      _is_enable = is_enabled;
    }

  private:
    bool _is_enable = false;
  } motor_state;

  const MotorControllerHardware::Parameter parameter_hardware = {
    {0x400, 0x480},
    16000,
    666ms,
    0.45f
  };
  const std::vector<eduart::robot::Rpm> rpm(2, 0);

  auto can_device = std::make_shared<CanCommunicationDevice>("eduart-can2", CanCommunicationDevice::CanType::CAN);
  auto communicator = std::make_shared<Communicator>(can_device, 1ms);
  auto executer = std::make_shared<Executer>(1ms);
  MotorControllerHardware controller("can_motor_controller", parameter_hardware, executer, communicator);

  // register callback for getting feedback
  controller.registerCallbackProcessMeasurementData(
    std::bind(&MotorState::callbackFeedback, &motor_state, std::placeholders::_1, std::placeholders::_2)
  );
  // use default parameter for initialization
  controller.initialize(std::vector<Motor::Parameter>(2));

  // start testing
  executer->start();

  SECTION("no timeout") {
    const auto stamp_start = system_clock::now();

    controller.processSetValue(rpm);
    controller.enable();
    std::this_thread::sleep_for(100ms); // wait for feedback

    while (duration_cast<milliseconds>(system_clock::now() - stamp_start) < 5000ms) {
      controller.processSetValue(rpm);
      REQUIRE(motor_state.isEnabled());
      std::this_thread::sleep_for(20ms); // ~50Hz
    }
  }

  SECTION("timeout") {
    const auto stamp_start = system_clock::now();

    controller.processSetValue(rpm);
    controller.enable();
    std::this_thread::sleep_for(100ms); // wait for feedback

    // first controller should be enabled for 1s
    while (duration_cast<milliseconds>(system_clock::now() - stamp_start) < 1000ms) {
      controller.processSetValue(rpm);
      REQUIRE(motor_state.isEnabled());
      std::this_thread::sleep_for(20ms); // ~50Hz
    }

    // timeout should occurred
    std::this_thread::sleep_for(700ms);
    REQUIRE_FALSE(motor_state.isEnabled());

    while (duration_cast<milliseconds>(system_clock::now() - stamp_start) < 5000ms) {
      controller.processSetValue(rpm);
      REQUIRE_FALSE(motor_state.isEnabled());
      std::this_thread::sleep_for(20ms); // ~50Hz
    }    
  }
}