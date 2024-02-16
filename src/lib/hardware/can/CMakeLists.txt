# Library Hardware CAN
add_library(eduart-hardware-can
  can_communication_device.cpp
  can_gateway_shield.cpp
  sensor_point_cloud_hardware.cpp
  hardware_component_factory.cpp
  motor_controller_hardware.cpp
  imu_sensor_hardware.cpp
)
target_link_libraries(eduart-hardware-can
  eduart-robot
  eduart-hardware
)
target_compile_features(eduart-hardware-can PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(eduart-hardware-can PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
