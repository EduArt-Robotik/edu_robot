# Tools
if(HAVE_MRAA)
  add_executable(uart_network_bridge
    uart_network_bridge.cpp
  )

  target_compile_features(uart_network_bridge PUBLIC c_std_99 cxx_std_17)
  target_include_directories(uart_network_bridge PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(uart_network_bridge
    mraa
  )
  target_compile_definitions(uart_network_bridge PUBLIC _WITH_MRAA)

  set(BUILT_TOOLS ${BUILT_TOOLS} "uart_network_bridge")
else()
  message(STATUS "Skipping uart_network_bridge: MRAA not found")
endif()

# Populating Built Nodes
set(BUILT_TOOLS ${BUILT_TOOLS} PARENT_SCOPE)
