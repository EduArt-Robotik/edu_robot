# Library Eduart IotBot Shield
add_library(eduart-ethernet-gateway
  ethernet_communicator.cpp
  ethernet_gateway_shield.cpp
)

target_link_libraries(eduart-ethernet-gateway
  # ${CPP_TYPESUPPORT_TARGET} needed for ROS distro humble and newer. TODO: handle this somehow!
  eduart-robot-eduard
)
target_compile_features(eduart-ethernet-gateway PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(eduart-ethernet-gateway PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# rosidl_target_interfaces(eduart-iotbot-shield ${PROJECT_NAME} "rosidl_typesupport_cpp")
