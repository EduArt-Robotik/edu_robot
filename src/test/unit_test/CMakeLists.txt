cmake_minimum_required(VERSION 3.8)

# Find Google Test
find_package(ament_cmake_gtest REQUIRED)

# Create test executable for Angle class
ament_add_gtest(test_angle
  test_angle.cpp
)

# Link test executable with required libraries
target_include_directories(test_angle PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(test_angle
  # Add any additional libraries if needed
)

# Set C++ standard
target_compile_features(test_angle PRIVATE cxx_std_17)

# Add compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(test_angle PRIVATE -Wall -Wextra -Wpedantic)
endif()
