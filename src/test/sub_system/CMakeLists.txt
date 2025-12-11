# Sub System Test for CAN Motor Controller
add_executable(sub_system_test_can_motor_controller sub_system_test_can_motor_controller.cpp)
#   target_include_directories(test_my_class PRIVATE include)
target_link_libraries(sub_system_test_can_motor_controller
  PRIVATE
    Catch2::Catch2WithMain
    eduart-hardware-can
)
ament_add_test(sub_system_test_can_motor_controller
  COMMAND sub_system_test_can_motor_controller
  TIMEOUT 30
)