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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
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
  "msg/SetLightingColor.msg"
  "msg/RobotStatusReport.msg"
# Services
  "srv/SetMode.srv"
  DEPENDENCIES
  std_msgs
)
## Line below needed for ROS distro humble and newer. TODO: handle this somehow!
# rosidl_get_typesupport_target(CPP_TYPESUPPORT_TARGET "${PROJECT_NAME}" "rosidl_typesupport_cpp")


## Build Library needed for ROS Nodes
# Library Eduart Robot
add_library(eduart-robot
  src/lib/edu_robot/robot.cpp
  src/lib/edu_robot/lighting.cpp
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
  "std_msgs"
  "geometry_msgs"
)
rosidl_target_interfaces(eduart-robot ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Library Eduart IotBot Shield
add_library(eduart-iotbot-shield
  src/lib/edu_iotbot_shield/iot_shield.cpp
  src/lib/edu_iotbot_shield/iot_shield_communicator.cpp
  src/lib/edu_iotbot_shield/lighting.cpp
  src/lib/edu_iotbot_shield/iotbot.cpp
)
if(HAVE_MRAA) ## TODO: use string for dependencies, so onyl one target_link_libraries is there!
  message("MRAA library found")
  target_link_libraries(eduart-iotbot-shield
    # ${CPP_TYPESUPPORT_TARGET} needed for ROS distro humble and newer. TODO: handle this somehow!
    eduart-robot
    mraa
  )
else()
  message("MRAA library not found")
  target_link_libraries(eduart-iotbot-shield
    # ${CPP_TYPESUPPORT_TARGET} needed for ROS distro humble and newer. TODO: handle this somehow!
    eduart-robot
  )
endif()
target_compile_features(eduart-iotbot-shield PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(eduart-iotbot-shield PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# rosidl_target_interfaces(eduart-iotbot-shield ${PROJECT_NAME} "rosidl_typesupport_cpp")

## Build Nodes
add_executable(iotbot-shield
  src/node/iotbot_shield_node.cpp
)
target_link_libraries(iotbot-shield
  eduart-iotbot-shield
  eduart-robot
)
ament_target_dependencies(iotbot-shield
  "rclcpp"
)
# rosidl_target_interfaces(iotbot-shield ${PROJECT_NAME} "rosidl_typesupport_cpp")

## Installing of Content
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS iotbot-shield eduart-iotbot-shield eduart-robot
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
