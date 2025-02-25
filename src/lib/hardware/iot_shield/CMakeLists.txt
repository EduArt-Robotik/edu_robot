# Library Eduart IotBot Shield
add_library(eduart-iotbot-shield
  iot_shield.cpp
  lighting_hardware.cpp
  motor_controller_hardware.cpp
  range_sensor_hardware.cpp
  imu_sensor_hardware.cpp
  iotbot_hardware_component_factory.cpp
  uart_communication_device.cpp
)
if(HAVE_MRAA) ## TODO: use string for dependencies, so onyl one target_link_libraries is there!
  message("MRAA library found")
  target_link_libraries(eduart-iotbot-shield
    "${cpp_typesupport_target}"
    eduart-bot
    mraa
  )
else()
  message("MRAA library not found")
  target_link_libraries(eduart-iotbot-shield
    "${cpp_typesupport_target}"
    eduart-bot
  )
endif()
target_compile_features(eduart-iotbot-shield PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(eduart-iotbot-shield PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(eduart-iotbot-shield
  eduart-robot
  eduart-hardware
)
list(APPEND BUILT_LIBRARIES eduart-iotbot-shield)

# Export hardware plugins
if (HAVE_MRAA)
  add_library(ros2-control-eduard-iot SHARED
    ros2_hardware_adapter.cpp
  )
  target_include_directories(ros2-control-eduard-iot PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(ros2-control-eduard-iot
    "pluginlib"
    "hardware_interface"
  )
  list(APPEND BUILT_LIBRARIES ros2-control-eduard-iot)
endif()

# Populating Built Libraries
set(BUILT_LIBRARIES ${BUILT_LIBRARIES} PARENT_SCOPE)