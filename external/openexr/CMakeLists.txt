# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) Contributors to the OpenEXR Project.

# We require this to get object library link library support
cmake_minimum_required(VERSION 3.14)

if(POLICY CMP0074)
  # enable find_package(<Package>) to use <Package>_ROOT as a hint
  cmake_policy(SET CMP0074 NEW)
endif()

if(POLICY CMP0077)
  # enable variables set outside to override options
  cmake_policy(SET CMP0077 NEW)
endif()

#######################################
# Create project and include cmake
# configuration files
#######################################

file(READ "src/lib/OpenEXRCore/openexr_version.h" VERSION_H)
string(REGEX MATCH "VERSION_MAJOR ([0-9]*)" _ ${VERSION_H})
set(OPENEXR_VERSION_MAJOR ${CMAKE_MATCH_1})
string(REGEX MATCH "VERSION_MINOR ([0-9]*)" _ ${VERSION_H})
set(OPENEXR_VERSION_MINOR ${CMAKE_MATCH_1})
string(REGEX MATCH "VERSION_PATCH ([0-9]*)" _ ${VERSION_H})
set(OPENEXR_VERSION_PATCH ${CMAKE_MATCH_1})

project(OpenEXR VERSION ${OPENEXR_VERSION_MAJOR}.${OPENEXR_VERSION_MINOR}.${OPENEXR_VERSION_PATCH} LANGUAGES C CXX)

set(OPENEXR_VERSION_RELEASE_TYPE "" CACHE STRING "Extra version tag string for OpenEXR build, such as -dev, -beta1, etc.")

set(OPENEXR_VERSION ${OpenEXR_VERSION})
set(OPENEXR_VERSION_API "${OpenEXR_VERSION_MAJOR}_${OpenEXR_VERSION_MINOR}")

# The SOVERSION (i.e. numerical component of SONAME) tracks the ABI
# version. Increment this number whenever, and only when, the ABI changes in
# non-backwards-compatible ways.
#
# The OpenEXR project policy is to append the library version 
# "major.minor.patch" to the SONAME to form the real shared library name.  
# For example, in "libOpenEXR.so.31.3.2.0", "libOpenEXR.so.31" is the SONAME
# and ".3.2.0" identifies the corresponding library release.

set(OPENEXR_LIB_SOVERSION 33)
set(OPENEXR_LIB_VERSION "") # e.g. "31.3.2.0"

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lib/Imf
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lib/OpenEXR
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lib/IlmThread
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lib/OpenEXRUtil
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lib/OpenEXRCore
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lib/Iex
  ${CMAKE_CURRENT_SOURCE_DIR}/../imath/src/
  ${CMAKE_CURRENT_SOURCE_DIR}/../imath/src/Imath/
  ${CMAKE_CURRENT_BINARY_DIR}/../imath
  ${CMAKE_CURRENT_BINARY_DIR}/../openjph/include/
)

include(cmake/LibraryDefine.cmake)
include(cmake/OpenEXRSetup.cmake)
add_subdirectory(cmake)
message(STATUS "Configure ${OPENEXR_PACKAGE_NAME}, library API version: ${OPENEXR_LIB_VERSION}")

# Hint: This can be set to enable custom find_package
# search paths, probably best to set it when configuring
# on the command line to cmake instead of setting it
# here.
###set(CMAKE_PREFIX_PATH "/prefix")

#######################################
# Add all source in subdirectories
#######################################

# Include these two modules without enable/disable options
add_subdirectory(src/lib)

# Tell CMake where to find the OpenEXRConfig.cmake file. Makes it possible to call
# find_package(OpenEXR) in downstream projects
set(OpenEXR_DIR "${CMAKE_CURRENT_BINARY_DIR}/cmake" CACHE PATH "" FORCE)

