# Findsensorring.cmake
#
# Finds edu_lib_sensorring.
#
# Provides:
#   sensorring_FOUND
#   sensorring::sensorring

include_guard(GLOBAL)
option(SENSORRING_USE_USBTINGO
  "Use USBTingo backend for edu_lib_sensorring"
  OFF
)


# 1. Prefer local checkout/submodule
set(_SENSORRING_LOCAL_DIR
    "${PROJECT_SOURCE_DIR}/external/edu_lib_sensorring"
)
if(EXISTS "${_SENSORRING_LOCAL_DIR}/CMakeLists.txt")
  message(STATUS
    "edu_lib_sensorring: using local copy at ${_SENSORRING_LOCAL_DIR}"
  )
  add_subdirectory(
    "${_SENSORRING_LOCAL_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/sensorring"
    EXCLUDE_FROM_ALL
  )

endif()


# 2. Try an installed package
if(NOT TARGET sensorring::sensorring)
  find_package(sensorring CONFIG QUIET)
endif()


# 3. Fetch fallback (only on Raspberry Pi/libgpiod systems)
if(NOT TARGET sensorring::sensorring)
  if(EXISTS "/usr/include/gpiod.hpp")
    message(STATUS
      "edu_lib_sensorring: Raspberry Pi detected, but edu_lib_sensorring was not found locally. Fetching it from GitHub..."
    )

    cmake_policy(PUSH)
    cmake_policy(SET CMP0135 NEW)
    include(FetchContent)
    FetchContent_Declare(
      sensorring_source
      URL
      https://github.com/EduArt-Robotik/edu_lib_sensorring/archive/refs/tags/v4.0.0-alpha.6.zip
    )
    FetchContent_MakeAvailable(sensorring_source)
    cmake_policy(POP)

  else()
    message(STATUS
      "edu_lib_sensorring: not found, hardware support disabled"
    )

  endif()
endif()


# Normalize target name and return result
if(TARGET sensorring AND NOT TARGET sensorring::sensorring)
  add_library(sensorring::sensorring ALIAS sensorring)
endif()

if(TARGET sensorring::sensorring)
  set(sensorring_FOUND TRUE)
  message(STATUS
    "edu_lib_sensorring: sensor ring hardware support enabled"
  )
else()
  set(sensorring_FOUND FALSE)
endif()

mark_as_advanced(sensorring_FOUND)
