add_library(processing-component
  collison_avoidance.cpp
)
target_link_libraries(processing-component
  # ${CPP_TYPESUPPORT_TARGET} needed for ROS distro humble and newer. TODO: handle this somehow!
)
target_compile_features(processing-component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(processing-component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  processing-component
  "rclcpp"
)
rosidl_target_interfaces(processing-component ${PROJECT_NAME} "rosidl_typesupport_cpp")
