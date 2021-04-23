# Copyright 2008-present Contributors to the OpenImageIO project.
# SPDX-License-Identifier: BSD-3-Clause
# https://github.com/OpenImageIO/oiio/blob/master/LICENSE.md

cmake_minimum_required (VERSION 3.12)
project (OpenImageIO VERSION 2.3.4.0
         HOMEPAGE_URL "https://openimageio.org"
         LANGUAGES CXX C)
set (PROJ_NAME OIIO)    # short name, caps
string (TOLOWER ${PROJ_NAME} PROJ_NAME_LOWER)  # short name lower case
string (TOUPPER ${PROJ_NAME} PROJ_NAME_UPPER)  # short name upper case
set (PROJECT_VERSION_RELEASE_TYPE "dev")   # "dev", "betaX", "RCY", ""
set (${PROJECT_NAME}_VERSION_RELEASE_TYPE ${PROJECT_VERSION_RELEASE_TYPE})
set (PROJECT_AUTHORS "Contributors to the OpenImageIO project")
set (${PROJECT_NAME}_SUPPORTED_RELEASE 0)  # Change to 1 for release branch
set (${PROJECT_NAME}_DEV_RELEASE 1)  # Change to 0 for release branch

# Identify whether this is included as a subproject of something else
if (NOT "${CMAKE_PROJECT_NAME}" STREQUAL "${PROJECT_NAME}")
    set (${PROJECT_NAME}_IS_SUBPROJECT ON)
    message (STATUS "${PROJECT_NAME} is configuring as a CMake subproject")
endif ()

set (PROJECT_VERSION_MAJOR 2)
set (PROJECT_VERSION_MINOR 3)
set (PROJECT_VERSION_PATCH 4)

if (NOT CMAKE_BUILD_TYPE)
    set (CMAKE_BUILD_TYPE "Release")
endif ()

message (STATUS "Configuring ${PROJECT_NAME} ${PROJECT_VERSION}")
message (STATUS "CMake ${CMAKE_VERSION}")
message (STATUS "CMake system           = ${CMAKE_SYSTEM}")
message (STATUS "CMake system name      = ${CMAKE_SYSTEM_NAME}")
message (STATUS "Project source dir     = ${PROJECT_SOURCE_DIR}")
message (STATUS "Project build dir      = ${CMAKE_BINARY_DIR}")
message (STATUS "Project install prefix = ${CMAKE_INSTALL_PREFIX}")
message (STATUS "Configuration types    = ${CMAKE_CONFIGURATION_TYPES}")
message (STATUS "Build type             = ${CMAKE_BUILD_TYPE}")

if ("${PROJECT_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
    message (FATAL_ERROR "Not allowed to run in-source build!")
endif ()

option (CMAKE_USE_FOLDERS "Use the FOLDER target property to organize targets into folders." ON)
mark_as_advanced (CMAKE_USE_FOLDERS)
if (CMAKE_USE_FOLDERS)
    set_property (GLOBAL PROPERTY USE_FOLDERS ON)
endif ()

set (OIIO_USING_IMATH 3)

option (VERBOSE "Print lots of messages while compiling" OFF)
option (${PROJ_NAME}_BUILD_TOOLS "Build the command-line tools" ON)
option (${PROJ_NAME}_BUILD_TESTS "Build the unit tests" ON)
set (OIIO_LIBNAME_SUFFIX "" CACHE STRING
     "Optional name appended to ${PROJECT_NAME} libraries that are built")
option (BUILD_OIIOUTIL_ONLY "If ON, will build *only* libOpenImageIO_Util" OFF)
set(USE_JPEGTURBO FALSE)
set(USE_OPENCV FALSE)
set(USE_OPENSSL FALSE)
set(USE_FREETYPE FALSE)
set(USE_GIF FALSE)
set(USE_PTEX FALSE)
set(USE_LIBRAW FALSE)
set(USE_FFMPEG FALSE)
set(EMBEDPLUGINS TRUE)
set (WEBP_FOUND TRUE)
set (LIBRAW_FOUND FALSE)
set (PNG_FOUND TRUE)
option (BUILD_COIIO "Build C interface library" ON)
set (PLUGIN_SEARCH_PATH "" CACHE STRING "Default plugin search path")
file (TO_NATIVE_PATH "${PLUGIN_SEARCH_PATH}" PLUGIN_SEARCH_PATH_NATIVE)
set (CMAKE_DEBUG_POSTFIX "" CACHE STRING "Library naming postfix for Debug builds (e.g., '_debug')")

option (OIIO_THREAD_ALLOW_DCLP "OIIO threads may use DCLP for speed" ON)
if (NOT OIIO_THREAD_ALLOW_DCLP)
    add_definitions ("-DOIIO_THREAD_ALLOW_DCLP=0")
endif ()

set (TEX_BATCH_SIZE "" CACHE STRING "Force TextureSystem SIMD batch size (e.g. 16)")
if (TEX_BATCH_SIZE)
    add_definitions ("-DOIIO_TEXTURE_SIMD_BATCH_WIDTH=${TEX_BATCH_SIZE}")
endif ()

# Set the default namespace
option (${PROJ_NAME}_NAMESPACE_INCLUDE_PATCH "Should the inner namespace include the patch number" OFF)
set (PROJ_NAMESPACE "OpenImageIO")
set (PROJ_NAMESPACE_V "${PROJ_NAMESPACE}_v${PROJECT_VERSION_MAJOR}_${PROJECT_VERSION_MINOR}")
set (${PROJ_NAME}_NAMESPACE ${PROJ_NAMESPACE_V} CACHE STRING
     "Customized outer namespace base name (version will be added)")
message(STATUS "Setting Namespace to: ${PROJ_NAMESPACE_V}")

INCLUDE_DIRECTORIES(
    "${CMAKE_CURRENT_SOURCE_DIR}/../libexpat/expat/lib"
    "${CMAKE_CURRENT_SOURCE_DIR}/../freetype2/include/"
    "${CMAKE_CURRENT_SOURCE_DIR}/../freetype2/src/include/"
    "${CMAKE_CURRENT_SOURCE_DIR}/../fontconfig/src/"
    "${CMAKE_CURRENT_SOURCE_DIR}/../fontconfig/include/"
    "${CMAKE_CURRENT_SOURCE_DIR}/../libjpeg/"
    "${CMAKE_CURRENT_BINARY_DIR}/../libjpeg/"
    "${CMAKE_CURRENT_SOURCE_DIR}/../libpng/"
    "${CMAKE_CURRENT_SOURCE_DIR}/../zlib"
    "${CMAKE_CURRENT_SOURCE_DIR}/../libiconv/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../webp/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../libtiff/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/../libtiff/port"
    "${CMAKE_CURRENT_BINARY_DIR}/../libtiff/src"
    "${CMAKE_CURRENT_BINARY_DIR}/../libtiff/port"
    "${CMAKE_CURRENT_SOURCE_DIR}/../libjasper"
    "${CMAKE_CURRENT_SOURCE_DIR}/../openexr"
    "${CMAKE_CURRENT_BINARY_DIR}/../openexr"
    "${CMAKE_CURRENT_SOURCE_DIR}/../openexr/Half"
    "${CMAKE_CURRENT_SOURCE_DIR}/../openexr/Iex"
    "${CMAKE_CURRENT_SOURCE_DIR}/../openexr/IlmImf"
    "${CMAKE_CURRENT_SOURCE_DIR}/../openexr/IlmThread"
    "${CMAKE_CURRENT_SOURCE_DIR}/../openexr/Imath"
    "${CMAKE_CURRENT_SOURCE_DIR}/../libboost/boost_1_67_0"
    "${CMAKE_CURRENT_SOURCE_DIR}/../fmt/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../robin-map/include"
)

# Check c++17
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wno-error=parentheses")

list (APPEND CMAKE_MODULE_PATH
      "${PROJECT_SOURCE_DIR}/src/cmake/modules"
      "${PROJECT_SOURCE_DIR}/src/cmake")

include (GNUInstallDirs)

# Utilities
include (colors)
include (check_is_enabled)
include (add_oiio_plugin)

# All the C++ and compiler related options and adjustments
include (compiler)

include_directories (
    BEFORE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/include"
    "${CMAKE_CURRENT_BINARY_DIR}/src/include"
    "${CMAKE_CURRENT_BINARY_DIR}/include"
    "${CMAKE_CURRENT_BINARY_DIR}/include/OpenImageIO"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/dds.imageio/squish"
  )

# Tell CMake to process the sub-directories
add_subdirectory (src/libutil)

# Add IO plugin directories -- if we are embedding plugins, we need to visit
# these directories BEFORE the OpenImageIO target is established (in
# src/libOpenImageIO). For each plugin, we append to the lists of source
# files, format libs, include directories, and definitions, all of which
# will be used by src/libOpenImageIO.
set (libOpenImageIO_srcs "")
set (format_plugin_libs "")
set (format_plugin_include_dirs "")
set (format_plugin_definitions "")
file (GLOB all_format_plugin_dirs src/*.imageio)
if ("${OIIO_SITE}" STREQUAL "SPI")
    # SPI only -- because of a workaround for a very weird linkage issue
    # specific to our system, we need to be sure libtiff is referenced first.
    file (GLOB tiff_format_plugin_dir src/tiff.imageio)
    list (REMOVE_ITEM all_format_plugin_dirs ${tiff_format_plugin_dir})
    list (INSERT all_format_plugin_dirs 0 ${tiff_format_plugin_dir})
endif ()

foreach (plugin_dir ${all_format_plugin_dirs})
  add_subdirectory (${plugin_dir})
endforeach ()

add_subdirectory (src/libOpenImageIO)

add_subdirectory (src/include)
