# Library Eduart IotBot Shield
add_library(eduart-ethernet-gateway
  ethernet_communication_device.cpp
  ethernet_request.cpp
  ethernet_gateway_shield.cpp
  hardware_component_factory.cpp
  imu_sensor_hardware.cpp
  lighting_hardware.cpp
  motor_controller_hardware.cpp
  range_sensor_hardware.cpp
)

target_link_libraries(eduart-ethernet-gateway
  "${cpp_typesupport_target}"
  # eduart-robot-eduard
  # eduart-robot-flex-bot
  eduart-robot
  eduart-hardware
)
target_compile_features(eduart-ethernet-gateway PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(eduart-ethernet-gateway PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
list(APPEND BUILT_LIBRARIES eduart-ethernet-gateway)

# Populating Built Libraries
set(BUILT_LIBRARIES ${BUILT_LIBRARIES} PARENT_SCOPE)