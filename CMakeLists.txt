cmake_minimum_required(VERSION 3.8)
project(edu_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++1z)# -O2)
endif()

## Dependencies
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# MRAA library
if(EXISTS "/usr/local/include/mraa/common.hpp")
  message(STATUS "Found mraa installed at ${MRAA_INCLUDE_DIR}.")
  add_compile_definitions(_WITH_MRAA)
  set(HAVE_MRAA "TRUE")
else()
  message(STATUS "Couldn't find mraa, are you compiling on a IOT 2050 device? IotBot's components will to be compiled.")
  set(HAVE_MRAA "FALSE")
endif()

# pigpio
if (EXISTS "/usr/include/gpiod.hpp")
  message(STATUS "Found libgpiod. Enable gpio hardware support for raspberry.")
  set(HAVE_GPIOD "TRUE")
else()
  message(STATUS "Couldn't find libgpiod. Disable hardware support for raspberry.")
  set(HAVE_GPIOD "FALSE")  
endif()

## Build Defined ROS Messages
rosidl_generate_interfaces(${PROJECT_NAME}
# Messages
  "msg/Mode.msg"
  "msg/State.msg"
  "msg/RobotState.msg"
  "msg/SetLightingColor.msg"
  "msg/RobotStatusReport.msg"
  "msg/Matrix.msg"
  "msg/RobotKinematicDescription.msg"
# Services
  "srv/SetMode.srv"
  "srv/GetKinematicDescription.srv"
  DEPENDENCIES
  std_msgs
)
# Create a target that can be used by all buildiung targets when the custom message are required for them.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Building Libraries
add_subdirectory(src/lib/action)
add_subdirectory(src/lib/algorithm)
add_subdirectory(src/lib/diagnostic)
add_subdirectory(src/lib/robot)
add_subdirectory(src/lib/processing_component)
add_subdirectory(src/lib/bot)
add_subdirectory(src/lib/hardware)

# Export hardware interface plugins
if (HAVE_MRAA)
  pluginlib_export_plugin_description_file(hardware_interface src/lib/hardware/iot_shield/ros2_control_eduard_iot.xml)
endif()
if (HAVE_GPIOD)
  pluginlib_export_plugin_description_file(hardware_interface src/lib/hardware/can_gateway/ros2_control_eduard_raspberry.xml)
endif()

# Building Nodes
add_subdirectory(src/node)

## Installing of Content
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY
    launch
    parameter
  DESTINATION share/${PROJECT_NAME}
)

message(STATUS "Nodes: ${BUILT_NODES}")
install(
  TARGETS
    ${BUILT_NODES}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  # for humble...
  # RUNTIME DESTINATION bin
  # for galactic...
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)
message(STATUS "Libraries: ${BUILT_LIBRARIES}")
install(
  TARGETS
    ${BUILT_LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${BUILT_LIBRARIES}
)
ament_export_dependencies(
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  diagnostic_updater
  Eigen3
  std_srvs
  hardware_interface
  pluginlib
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_package()
