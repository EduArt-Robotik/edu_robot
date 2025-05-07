# Library Hardware CAN
add_library(eduart-hardware-can
  can_communication_device.cpp
  can_gateway_shield.cpp
  sensor_tof_hardware.cpp
  sensor_tof_ring_hardware.cpp
  hardware_component_factory.cpp
  motor_controller_hardware.cpp
  imu_sensor_hardware.cpp
  lighting_hardware.cpp
  sensor_virtual_range.cpp
  sensor_point_cloud_fusion.cpp
)
target_link_libraries(eduart-hardware-can
  eduart-algorithm
  eduart-robot
  eduart-hardware
)
target_compile_features(eduart-hardware-can PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(eduart-hardware-can PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(eduart-hardware-can
  "tf2"
  "tf2_ros"
  "tf2_sensor_msgs"
)
list(APPEND BUILT_LIBRARIES eduart-hardware-can)

# Export hardware plugins
if (HAVE_GPIOD)
  add_library(ros2-control-eduard-raspberry SHARED
    ros2_hardware_adapter.cpp
  )
  target_include_directories(ros2-control-eduard-raspberry PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(ros2-control-eduard-raspberry
    gpiodcxx
    gpiod
  )
  ament_target_dependencies(ros2-control-eduard-raspberry
    "pluginlib"
    "hardware_interface"
  )
  list(APPEND BUILT_LIBRARIES ros2-control-eduard-raspberry)
endif()

# Populating Built Libraries
set(BUILT_LIBRARIES ${BUILT_LIBRARIES} PARENT_SCOPE)
