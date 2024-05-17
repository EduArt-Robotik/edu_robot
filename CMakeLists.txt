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

add_subdirectory(src/lib/action)
add_subdirectory(src/lib/diagnostic)
add_subdirectory(src/lib/robot)
add_subdirectory(src/lib/processing_component)
add_subdirectory(src/lib/bot)
add_subdirectory(src/lib/hardware)
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

install(
  TARGETS
    iotbot-shield
    ethernet-gateway
    ethernet-gateway-universal-bot
    ethernet-gateway-ohmni-bot
    igus-can-turtle-bot
    can-gateway-universal-bot
    can-gateway-bot
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  # for humble...
  # RUNTIME DESTINATION bin
  # for galactic...
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)
install(
  TARGETS
    eduart-robot-universal-bot
    eduart-iotbot-shield
    eduart-robot-eduard
    eduart-robot
    processing-component
    eduart-ethernet-gateway
    ohmni-bot
    eduart-action
    eduart-diagnostic
    eduart-turtle
    igus
    eduart-hardware
    eduart-hardware-can
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
  eduart-iotbot-shield
  eduart-robot-eduard
  eduart-robot
  processing-component
  eduart-ethernet-gateway
  eduart-robot-universal-bot
  eduart-action
)
ament_export_dependencies(
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  diagnostic_updater
  Eigen3
  std_srvs
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_package()
