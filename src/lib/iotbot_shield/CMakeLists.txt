# Library Eduart IotBot Shield
add_library(eduart-iotbot-shield
  iot_shield.cpp
  iot_shield_communicator.cpp
  lighting_hardware.cpp
  iotbot.cpp
  motor_controller_hardware.cpp
  range_sensor_hardware.cpp
  imu_sensor_hardware.cpp
  iotbot_hardware_component_factory.cpp
)
if(HAVE_MRAA) ## TODO: use string for dependencies, so onyl one target_link_libraries is there!
  message("MRAA library found")
  target_link_libraries(eduart-iotbot-shield
    "${cpp_typesupport_target}"
    eduart-robot-eduard
    mraa
  )
else()
  message("MRAA library not found")
  target_link_libraries(eduart-iotbot-shield
    "${cpp_typesupport_target}"
    eduart-robot-eduard
  )
endif()
target_compile_features(eduart-iotbot-shield PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(eduart-iotbot-shield PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
