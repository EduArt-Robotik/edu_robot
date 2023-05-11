## Build Library needed for ROS Nodes
# Library Eduart Robot

add_library(eduart-robot
  robot.cpp
  lighting.cpp
  motor_controller.cpp
  sensor.cpp
  range_sensor.cpp
  imu_sensor.cpp
)
target_link_libraries(eduart-robot
  Eigen3::Eigen
  processing-component
  "${cpp_typesupport_target}"
)
target_compile_features(eduart-robot PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(eduart-robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  eduart-robot
  "rclcpp"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "nav_msgs"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
)
# Below ROS humble this line is needed...
# rosidl_target_interfaces(eduart-robot ${PROJECT_NAME} "rosidl_typesupport_cpp")
