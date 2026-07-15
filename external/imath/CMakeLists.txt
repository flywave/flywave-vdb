# SPDX-License-Identifier: BSD-3-Clause
# Copyright Contributors to the OpenEXR Project.

cmake_minimum_required(VERSION 3.14)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(POLICY CMP0077)
  # enable variables set outside to override options
  cmake_policy(SET CMP0077 NEW)
endif()

# Imath version

project(Imath VERSION 3.2.2 LANGUAGES C CXX)

set(IMATH_VERSION_RELEASE_TYPE "" CACHE STRING "Extra version tag string for Imath build, such as -dev, -beta1, etc.")

set(IMATH_VERSION ${Imath_VERSION})
set(IMATH_VERSION_API "${Imath_VERSION_MAJOR}_${Imath_VERSION_MINOR}")

# The SOVERSION (i.e. numerical component of SONAME) tracks the ABI
# version. Increment this number whenever, and only when, the ABI changes in
# non-backwards-compatible ways.
#
# The OpenEXR project policy is to append the library version
# "major.minor.patch" to the SONAME to form the real shared library name.
# For example, in "libImath.so.31.3.2.0", "libImath.so.31" is the SONAME
# and ".3.2.0" identifies the corresponding library release.

set(IMATH_LIB_SOVERSION 30)
set(IMATH_LIB_VERSION "${IMATH_LIB_SOVERSION}.${IMATH_VERSION}") # e.g. "30.3.2.0"

# ImathSetup.cmake declares all the configuration variables visible
# in cmake-gui or similar and the rest of the global
# project setup. Check the context to see what is configurable.
include(config/ImathSetup.cmake)

message(STATUS "Configure ${IMATH_PACKAGE_NAME}, library API version: ${IMATH_LIB_VERSION}")

option(IMATH_INSTALL "Install Imath library" OFF)

if (APPLE)
  option(IMATH_BUILD_APPLE_FRAMEWORKS "Build as Apple Frameworks" OFF)
endif()

# Config headers and package config files
add_subdirectory(config)

# Source code is in src/Imath
add_subdirectory(src/Imath)

# Imath_DIR points to the location of ImathConfig.cmake, which tells
# downstream projects where to find Imath, via find_package(Imath).
set(Imath_DIR "${CMAKE_CURRENT_BINARY_DIR}/config" CACHE PATH "" FORCE)
