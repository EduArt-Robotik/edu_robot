# Library Algorithm
add_library(eduart-robot-algorithm
  odometry_kalman_filter_model.cpp
)
target_include_directories(eduart-robot-algorithm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(eduart-robot-algorithm
  Eigen3::Eigen
)
