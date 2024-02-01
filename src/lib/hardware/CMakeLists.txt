# Library Hardware
add_library(eduart-hardware
  communicator.cpp
)
target_link_libraries(eduart-hardware
  eduart-robot
)
target_compile_features(eduart-hardware PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(eduart-hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Including Subdirectories
add_subdirectory(ethernet_gateway)
add_subdirectory(iotbot_shield)
add_subdirectory(igus)
add_subdirectory(can)
