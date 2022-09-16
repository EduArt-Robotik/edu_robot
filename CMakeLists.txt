cmake_minimum_required(VERSION 3.8)
project(edu_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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
find_package(nav_msgs REQUIRED)

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
# Services
  "srv/SetMode.srv"
)
## Line below needed for ROS distro humble and newer. TODO: handle this somehow!
# rosidl_get_typesupport_target(CPP_TYPESUPPORT_TARGET "${PROJECT_NAME}" "rosidl_typesupport_cpp")


## Build Library needed for ROS Nodes
add_library(eduart-robot
  src/robot.cpp
  src/iot_shield/iot_shield.cpp
  src/iot_shield/iot_shield_communicator.cpp
  src/iot_shield/lighting.cpp
)
target_link_libraries(eduart-robot
  # ${CPP_TYPESUPPORT_TARGET} needed for ROS distro humble and newer. TODO: handle this somehow!
)
target_compile_features(eduart-robot PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(eduart-robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  eduart-robot
  "rclcpp"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "nav_msgs"
)
rosidl_target_interfaces(eduart-robot ${PROJECT_NAME} "rosidl_typesupport_cpp")


## Build Nodes

## Installing of Content
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS eduart-robot
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
  eduart-robot
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
