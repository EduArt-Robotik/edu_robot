# Library for Igus Motor Controller Hardware
add_library(igus
  hardware_component_factory.cpp
  motor_controller_hardware.cpp
  can_gateway_shield.cpp
)
target_include_directories(igus PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(igus
  "${cpp_typesupport_target}"
  eduart-robot
  eduart-hardware
  eduart-hardware-can
)
ament_target_dependencies(igus
  "rclcpp"
)
list(APPEND BUILT_LIBRARIES igus)

# Populating Built Libraries
set(BUILT_LIBRARIES ${BUILT_LIBRARIES} PARENT_SCOPE)