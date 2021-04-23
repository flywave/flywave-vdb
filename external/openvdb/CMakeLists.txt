# Copyright Contributors to the OpenVDB Project
# SPDX-License-Identifier: MPL-2.0
#
#[=======================================================================[

  CMake Configuration for OpenVDB

  The OpenVDB CMake build system generates targets depending on the
  enabled components. It is designed for out of source CMake generation
  (a build location for CMake to write to will be required). For example,
  from the root of the repository:

    mkdir build
    cd build
    cmake ../

  Depending on the components you choose to build, a number of optional
  and required dependencies are expected. See the dependency documentation
  for more information:

    https://academysoftwarefoundation.github.io/openvdb/dependencies.html

  And the documentation on building OpenVDB for more in depth installation
  instructions:

    https://academysoftwarefoundation.github.io/openvdb/build.html

  This CMakeLists file provides most available options for configuring the
  build and installation of all OpenVDB components. By default the core
  library and the vdb_print binary are enabled.

  Note that various packages have inbuilt CMake module support. See the
  CMake documentation for more ZLib, Doxygen, OpenGL, Boost and Python
  controls:

    https://cmake.org/cmake/help/latest/manual/cmake-modules.7.html

  OpenVDB's CMake supports building the various components of against a
  prior installation of OpenVDB.

#]=======================================================================]

# note: cmake_minimum_required must be called before project commands to
#  ensure policy scope is set up correctly
cmake_minimum_required(VERSION 3.12)

# CMP0091 allows for MSVC ABI targetting via CMAKE_MSVC_RUNTIME_LIBRARY
# from CMake 3.15 and above. Must come before project().
if(POLICY CMP0091)
  cmake_policy(SET CMP0091 NEW)
endif()

project(OpenVDB LANGUAGES CXX)

###### OpenVDB Build/Component Options

include(CMakeDependentOption)
include(GNUInstallDirs)

set(_OPENVDB_SIMD_OPTIONS None SSE42 AVX)
if(NOT OPENVDB_SIMD)
  set(OPENVDB_SIMD None CACHE STRING
    "Choose whether to enable SIMD compiler flags or not, options are: None SSE42 AVX.
    Although not required, it is strongly recommended to enable SIMD. AVX implies SSE42.
    None is the default." FORCE
  )
elseif(NOT ${OPENVDB_SIMD} IN_LIST _OPENVDB_SIMD_OPTIONS)
  message(WARNING "Unrecognized or unsupported value for OPENVDB_SIMD, "
    "using None instead.")
  set(OPENVDB_SIMD None CACHE STRING FORCE)
endif()

if(USE_BLOSC AND NOT USE_ZLIB)
  message(WARNING "ZLib can only be disabled if Blosc is also disabled. Enabling ZLib.")
endif()

# Various root level CMake options which are marked as advanced

set(USE_LOG4CPLUS FALSE)
set(USE_ZLIB TRUE)
set(USE_BLOSC TRUE)
set(USE_IMATH_HALF TRUE)
set(OPENVDB_USE_DEPRECATED_ABI_6 FALSE)
set(OPENVDB_USE_FUTURE_ABI_9 FALSE)
set(OPENVDB_FUTURE_DEPRECATION FALSE)
set(CONCURRENT_MALLOC Tbbmalloc)
set(USE_COLORED_OUTPUT FALSE)

mark_as_advanced(
  OPENVDB_CXX_STRICT
  OPENVDB_ENABLE_RPATH
  USE_HOUDINI
  USE_MAYA
  USE_LOG4CPLUS
  USE_IMATH_HALF
  USE_CCACHE
  OPENVDB_BUILD_HOUDINI_ABITESTS
  DISABLE_DEPENDENCY_VERSION_CHECKS
  DISABLE_CMAKE_SEARCH_PATHS
  OPENVDB_USE_DEPRECATED_ABI_6
  OPENVDB_USE_FUTURE_ABI_9
  OPENVDB_FUTURE_DEPRECATION
  CONCURRENT_MALLOC
  USE_COLORED_OUTPUT
  SYSTEM_LIBRARY_PATHS
  OPENVDB_SIMD
)

# Configure minimum version requirements - some are treated specially and fall
# outside of the DISABLE_DEPENDENCY_VERSION_CHECKS catch

# @note  Blosc version is currently treated as exception which must be adhered
# to. The minimum version must be at least 1.5.0. Previous versions are incompatible.
# Later versions (including 1.5.4), can be buggy on certain platforms.
set(MINIMUM_BLOSC_VERSION 1.5.0)
# @note  ABI always enforced so the correct deprecation messages are available.
# OPENVDB_USE_DEPRECATED_ABI_<VERSION> should be used to circumvent this
set(MINIMUM_OPENVDB_ABI_VERSION 6)
set(MINIMUM_CXX_STANDARD 14)
set(FUTURE_MINIMUM_OPENVDB_ABI_VERSION 7)
set(FUTURE_OPENVDB_ABI_VERSION 9)

if(NOT DISABLE_DEPENDENCY_VERSION_CHECKS)
  # @note  Currently tracking CY2019 of the VFX platform where available
  set(MINIMUM_GCC_VERSION 6.3.1)
  set(MINIMUM_CLANG_VERSION 3.8)
  set(MINIMUM_ICC_VERSION 17)
  set(MINIMUM_MSVC_VERSION 19.10)

  set(MINIMUM_BOOST_VERSION 1.61)  # @warning should be 1.66, but H18 ships with 1.61
  set(MINIMUM_ILMBASE_VERSION 2.2) # @warning should be 2.3, but H18 ships with 2.2
  set(MINIMUM_OPENEXR_VERSION 2.2) # @warning should be 2.3, but H18 ships with 2.2
  set(MINIMUM_ZLIB_VERSION 1.2.7)
  set(MINIMUM_TBB_VERSION 2018.0)
  set(MINIMUM_LLVM_VERSION 6.0.0)

  set(MINIMUM_PYTHON_VERSION 2.7)
  set(MINIMUM_NUMPY_VERSION 1.14.0)

  set(MINIMUM_GOOGLETEST_VERSION 1.8)
  set(MINIMUM_GLFW_VERSION 3.1)
  set(MINIMUM_LOG4CPLUS_VERSION 1.1.2)
  set(MINIMUM_HOUDINI_VERSION 18.0)

  # These always promote warnings rather than errors
  set(MINIMUM_MAYA_VERSION 2017)
  set(MINIMUM_DOXYGEN_VERSION 1.8.8)
endif()

# VFX 20 deprecations to transition to MINIMUM_* variables in OpenVDB 9.0.0

# Note: CMake 3.15 chosen as possible next version due to Windows support
# with CMAKE_MSVC_RUNTIME_LIBRARY, numpy modules with CMake 3.14 and simplified
# python linkage on macOS (Python::Module target in 3.15)
set(FUTURE_MINIMUM_CMAKE_VERSION 3.15)

# No compiler upgrades planned
#set(FUTURE_MINIMUM_GCC_VERSION 6.3.1)
#set(FUTURE_MINIMUM_ICC_VERSION 17)
#set(FUTURE_MINIMUM_MSVC_VERSION 19.10)

set(FUTURE_MINIMUM_ILMBASE_VERSION 2.4)
set(FUTURE_MINIMUM_OPENEXR_VERSION 2.4)
set(FUTURE_MINIMUM_BOOST_VERSION 1.70)
set(FUTURE_MINIMUM_TBB_VERSION 2019.0)
set(FUTURE_MINIMUM_PYTHON_VERSION 3.7)
set(FUTURE_MINIMUM_NUMPY_VERSION 1.17.0)
set(FUTURE_MINIMUM_HOUDINI_VERSION 18.5)

#########################################################################

# General CMake and CXX settings

if(FUTURE_MINIMUM_CMAKE_VERSION)
  if(${CMAKE_VERSION} VERSION_LESS ${FUTURE_MINIMUM_CMAKE_VERSION})
    message(DEPRECATION "Support for CMake versions < ${FUTURE_MINIMUM_CMAKE_VERSION} "
      "is deprecated and will be removed.")
  endif()
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD ${MINIMUM_CXX_STANDARD} CACHE STRING
    "The C++ standard whose features are requested to build OpenVDB components." FORCE)
elseif(CMAKE_CXX_STANDARD LESS ${MINIMUM_CXX_STANDARD})
  message(FATAL_ERROR "Provided C++ Standard is less than the supported minimum."
    "Required is at least \"${MINIMUM_CXX_STANDARD}\" (found ${CMAKE_CXX_STANDARD})")
endif()

# Configure MS Runtime

if(CMAKE_MSVC_RUNTIME_LIBRARY AND CMAKE_VERSION VERSION_LESS 3.15)
  # CMAKE_MSVC_RUNTIME_LIBRARY selects the MSVC runtime library for use by compilers
  # targeting the MSVC ABI. Options are: MultiThreaded MultiThreadedDLL MultiThreadedDebug
  # MultiThreadedDebugDLL. If empty, CMake defaults to MultiThreaded$<$<CONFIG:Debug>:Debug>DLL
  message(FATAL_ERROR "CMAKE_MSVC_RUNTIME_LIBRARY support is only available from CMake 3.15")
endif()
if(WIN32 AND CMAKE_MSVC_RUNTIME_LIBRARY)
  message(STATUS "CMAKE_MSVC_RUNTIME_LIBRARY set to target ${CMAKE_MSVC_RUNTIME_LIBRARY}")

  # Configure Boost library varient on Windows
  if(NOT Boost_USE_STATIC_RUNTIME)
    set(Boost_USE_STATIC_RUNTIME OFF)
    if(CMAKE_MSVC_RUNTIME_LIBRARY STREQUAL MultiThreaded OR
       CMAKE_MSVC_RUNTIME_LIBRARY STREQUAL MultiThreadedDebug)
      set(Boost_USE_STATIC_RUNTIME ON)
    endif()
  endif()
  if(NOT Boost_USE_DEBUG_RUNTIME)
    set(Boost_USE_DEBUG_RUNTIME OFF)
    if(CMAKE_MSVC_RUNTIME_LIBRARY STREQUAL MultiThreadedDebugDLL OR
       CMAKE_MSVC_RUNTIME_LIBRARY STREQUAL MultiThreadedDebug)
      set(Boost_USE_DEBUG_RUNTIME ON)
    endif()
  endif()
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

if(OPENVDB_ENABLE_RPATH)
  # Configure rpath for installation base on the following:
  # https://gitlab.kitware.com/cmake/community/wikis/doc/cmake/RPATH-handling
  set(CMAKE_SKIP_BUILD_RPATH FALSE)
  set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()

# For CMake's find Threads module which brings in pthread - This flag
# forces the compiler -pthread flag vs -lpthread
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

# Add our cmake modules

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Add backports to support older versions of CMake
#  FindNumPy.cmake is needed if CMake < 3.14

if(${CMAKE_VERSION} VERSION_LESS 3.14)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/backports")
endif()

# Add cmake modules to installation command
# @todo fix our glew cmake module

include(cmake/config/OpenVDBBuildTypes.cmake)

if(CMAKE_BUILD_TYPE EQUAL coverage)
  # use .gcno extension instead of .cc.gcno
  # @note This is an undocumented internal cmake var and does not work
  # with multi config generators
  set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE 1)
endif()

# CMAKE_BUILD_TYPE is ignored for multi config generators i.e. MSVS
get_property(_isMultiConfig GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT _isMultiConfig)
  message(STATUS "CMake Build Type: ${CMAKE_BUILD_TYPE}")
endif()

#########################################################################

# Compiler configuration. Add definitions for a number of compiler warnings
# for sub projects and verify version requirements
# @todo  add definitions for Intel.

if(OPENVDB_SIMD STREQUAL "AVX")
  add_compile_options(-mavx -msse4.2)
  add_definitions(-DOPENVDB_USE_AVX)
  add_definitions(-DOPENVDB_USE_SSE42)
elseif(OPENVDB_SIMD STREQUAL "SSE42")
  add_compile_options(-msse4.2)
  add_definitions(-DOPENVDB_USE_SSE42)
endif()

#########################################################################

# Configure our cmake modules to only search for static libraries
set(BLOSC_USE_STATIC_LIBS ON)
set(OPENEXR_USE_STATIC_LIBS ON)
set(ILMBASE_USE_STATIC_LIBS ON)
set(TBB_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC_LIBS ON)

# Configure OpenVDB Library and ABI versions
include("${CMAKE_CURRENT_LIST_DIR}/cmake/OpenVDBUtils.cmake")
set(OPENVDB_VERSION_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/openvdb/openvdb/version.h")
if(NOT EXISTS "${OPENVDB_VERSION_HEADER}")
  message(FATAL_ERROR "Unable to read ${OPENVDB_VERSION_HEADER}. File does not exist.")
endif()

OPENVDB_VERSION_FROM_HEADER("${OPENVDB_VERSION_HEADER}"
  VERSION OpenVDB_VERSION
  MAJOR   OpenVDB_MAJOR_VERSION
  MINOR   OpenVDB_MINOR_VERSION
  PATCH   OpenVDB_PATCH_VERSION
)
message(STATUS "Configuring for OpenVDB Version ${OpenVDB_VERSION}")

# Validate the OpenVDB ABI Version. If OpenVDB_ABI is not set, we're either building
# the core library OR the ABI hasn't been deduced from a VDB installation. Use the
# value from OPENVDB_ABI_VERSION_NUMBER, falling back to the lib major version number

if(NOT OpenVDB_ABI)
  if(OPENVDB_ABI_VERSION_NUMBER)
    set(OpenVDB_ABI ${OPENVDB_ABI_VERSION_NUMBER})
  else()
    set(OpenVDB_ABI ${OpenVDB_MAJOR_VERSION})
  endif()
endif()

# From the deduced ABI, check against the required ABI for Houdini (if set).
# Forcefully set the ABI to the required value if necessary - do this after to
# explicitly warn the user if their chosen value is different.

add_definitions(-DOPENVDB_ABI_VERSION_NUMBER=${OpenVDB_ABI})
message(STATUS "Configuring for OpenVDB ABI Version ${OpenVDB_ABI}")

# Always force set as we may need to change it if it's incompatible with Houdini
set(OPENVDB_ABI_VERSION_NUMBER ${OpenVDB_ABI} CACHE STRING [=[
Build for compatibility with version N of the OpenVDB Grid ABI, where N is 3, 4, 5 etc. (some newer features
will be disabled). If OPENVDB_BUILD_CORE is OFF, CMake attempts to query the installed vdb_print binary to
determine the ABI number. You may set this to force a given ABI number.]=] FORCE)

##########################################################################

INCLUDE_DIRECTORIES(
    "${CMAKE_CURRENT_SOURCE_DIR}/openvdb"
    "${CMAKE_CURRENT_BINARY_DIR}/openvdb/openvdb"
    "${CMAKE_CURRENT_SOURCE_DIR}/../tbb/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../openexr/"
    "${CMAKE_CURRENT_SOURCE_DIR}/../c-blosc/"
    "${CMAKE_CURRENT_SOURCE_DIR}/../libboost/boost_1_67_0/"
)

add_subdirectory(openvdb/openvdb)
