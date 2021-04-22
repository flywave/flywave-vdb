cmake_minimum_required(VERSION 2.8)
project(blosc)

# parse the full version numbers from blosc.h
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/blosc/blosc.h _blosc_h_contents)
string(REGEX REPLACE ".*#define[ \t]+BLOSC_VERSION_MAJOR[ \t]+([0-9]+).*"
     "\\1" BLOSC_VERSION_MAJOR ${_blosc_h_contents})
string(REGEX REPLACE ".*#define[ \t]+BLOSC_VERSION_MINOR[ \t]+([0-9]+).*"
    "\\1" BLOSC_VERSION_MINOR ${_blosc_h_contents})
string(REGEX REPLACE ".*#define[ \t]+BLOSC_VERSION_RELEASE[ \t]+([0-9]+).*"
    "\\1" BLOSC_VERSION_PATCH ${_blosc_h_contents})
string(REGEX REPLACE ".*#define[ \t]+BLOSC_VERSION_STRING[ \t]+\"([-0-9A-Za-z.]+)\".*"
    "\\1" BLOSC_VERSION_STRING ${_blosc_h_contents})

message("Configuring for Blosc version: " ${BLOSC_VERSION_STRING})

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set(HAVE_LZ4 TRUE)
set(HAVE_SNAPPY TRUE)
set(HAVE_ZLIB TRUE)
set(HAVE_ZSTD TRUE)

if(NOT PREFER_EXTERNAL_COMPLIBS)
    message(STATUS "Finding external libraries disabled.  Using internal sources.")
endif(NOT PREFER_EXTERNAL_COMPLIBS)

# create the config.h file
configure_file ("blosc/config.h.in"  "${CMAKE_CURRENT_BINARY_DIR}/blosc/config.h" )
# now make sure that you set the build directory on your "Include" path when compiling
include_directories("${CMAKE_CURRENT_BINARY_DIR}/blosc/")

# flags
# @TODO: set -Wall
# @NOTE: -O3 is enabled in Release mode (CMAKE_BUILD_TYPE="Release")

# Set the "-msse2" build flag only if the CMAKE_C_FLAGS is not already set.
# Probably "-msse2" should be appended to CMAKE_C_FLAGS_RELEASE.
find_package(SSE)
if(CMAKE_C_COMPILER_ID STREQUAL GNU OR CMAKE_C_COMPILER_ID STREQUAL Clang)
     if(NOT CMAKE_C_FLAGS AND SSE2_TRUE)
         message(STATUS "SSE2 is here.  Adding support for it.")
         set(CMAKE_C_FLAGS -msse2 CACHE STRING "C flags." FORCE)
     endif(NOT CMAKE_C_FLAGS AND SSE2_TRUE)
endif(CMAKE_C_COMPILER_ID STREQUAL GNU OR CMAKE_C_COMPILER_ID STREQUAL Clang)

if(MSVC)
    if(NOT CMAKE_C_FLAGS)
        set(CMAKE_C_FLAGS "/Ox" CACHE STRING "C flags." FORCE)
    endif(NOT CMAKE_C_FLAGS)
endif(MSVC)

if(WIN32)
    # For some supporting headers
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/blosc")
endif(WIN32)


# subdirectories
add_subdirectory(blosc)
