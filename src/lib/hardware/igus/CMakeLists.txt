# Library for Igus Motor Controller Hardware
add_library(igus
  can_communicator.cpp
  can_gateway_shield.cpp
  hardware_component_factory.cpp
  motor_controller_hardware.cpp
)
target_include_directories(igus PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(igus
  "${cpp_typesupport_target}"
  eduart-robot
)
ament_target_dependencies(igus
  "rclcpp"
)