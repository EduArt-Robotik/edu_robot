add_library(processing-component
  collison_avoidance.cpp
  odometry_estimator.cpp
  processing_component.cpp
  shutting_downer.cpp
)
target_link_libraries(processing-component
  Eigen3::Eigen
  "${cpp_typesupport_target}"
  rclcpp::rclcpp
  "${nav_msgs_TARGETS}"
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)
target_compile_features(processing-component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(processing-component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
list(APPEND BUILT_LIBRARIES processing-component)

# Populating Built Libraries
set(BUILT_LIBRARIES ${BUILT_LIBRARIES} PARENT_SCOPE)