cmake_minimum_required(VERSION 3.8)
project(edu_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# MRAA library
if(EXISTS "/usr/include/mraa/common.hpp")
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
# Services
  "srv/SetMode.srv"
  DEPENDENCIES
  std_msgs
)
## Line below needed for ROS distro humble and newer. TODO: handle this somehow!
# rosidl_get_typesupport_target(CPP_TYPESUPPORT_TARGET "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_subdirectory(src/lib/robot)
add_subdirectory(src/lib/eduard)
add_subdirectory(src/lib/iotbot_shield)
add_subdirectory(src/lib/ethernet_gateway)
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
  TARGETS iotbot-shield eduart-iotbot-shield eduart-robot-eduard eduart-robot eduart-ethernet-gateway
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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

ament_package()
