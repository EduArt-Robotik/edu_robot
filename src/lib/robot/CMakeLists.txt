## Build Library needed for ROS Nodes
# Library Eduart Robot

add_library(eduart-robot
  robot.cpp
  lighting.cpp
  motor_controller.cpp
  sensor.cpp
  sensor_range.cpp
  sensor_imu.cpp
  sensor_point_cloud.cpp
  motor.cpp
  executer.cpp
)
target_link_libraries(eduart-robot
  Eigen3::Eigen
  processing-component
  eduart-action
  eduart-diagnostic
  "${cpp_typesupport_target}"
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  "${nav_msgs_TARGETS}"
  "${std_msgs_TARGETS}"
  "${geometry_msgs_TARGETS}"
  "${sensor_msgs_TARGETS}"
  diagnostic_updater::diagnostic_updater
  "${diagnostic_msgs_TARGETS}"
  "${std_srvs_TARGETS}"
)
target_compile_features(eduart-robot PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(eduart-robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
list(APPEND BUILT_LIBRARIES eduart-robot)

# Populating Built Libraries
set(BUILT_LIBRARIES ${BUILT_LIBRARIES} PARENT_SCOPE)