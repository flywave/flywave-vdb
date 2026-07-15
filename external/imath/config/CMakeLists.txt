# SPDX-License-Identifier: BSD-3-Clause
# Copyright Contributors to the OpenEXR Project.

if (IMATH_ENABLE_LARGE_STACK)
  set(IMATH_HAVE_LARGE_STACK ON)
endif()
if (IMATH_USE_DEFAULT_VISIBILITY)
  set(IMATH_ENABLE_API_VISIBILITY OFF)
else()
  set(IMATH_ENABLE_API_VISIBILITY ON)
endif()

configure_file(ImathConfig.h.in ${CMAKE_CURRENT_BINARY_DIR}/../ImathConfig.h)
set(IMATH_CONFIG_DIR "${CMAKE_CURRENT_BINARY_DIR}/../" CACHE INTERNAL "")

add_library(ImathConfig INTERFACE)
add_library(Imath::Config ALIAS ImathConfig)

target_include_directories(ImathConfig INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${IMATH_OUTPUT_SUBDIR}>)

