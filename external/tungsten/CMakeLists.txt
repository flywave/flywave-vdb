cmake_minimum_required(VERSION 2.8.11)
project(Tungsten)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

include(CXX11)
check_for_cxx11_compiler(CXX11_COMPILER)

if(CXX11_COMPILER)
    enable_cxx11()
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} seems to have no C++11 support. Please try again with a more recent compiler version.")
endif()

# AVX does not do much benefit at the moment, but breaks compilation on some platforms.
# Disabled for now until AVX becomes important enough to reconsider.
SET(EMBREE_MAX_ISA "SSE4.2" CACHE STRING "Selects highest ISA to support.")
set(USE_AVX FALSE CACHE BOOL "Use AVX.")

include(OptimizeForArchitecture)
OptimizeForArchitecture()

if (MSVC)
    # Needed by MSVC, but not added by OptimizeForArchitexture()
    add_definitions(-D__SSE__)
endif()

add_definitions(-DINSTALL_PREFIX="${CMAKE_INSTALL_PREFIX}")

if (USE_AVX)
    message(STATUS "Compiling with AVX support")
    set(__AVX__ 1)
elseif (USE_SSE4_2)
    message(STATUS "Compiling with SSE4.2 support")
elseif (USE_SSSE3)
    message(STATUS "Compiling with SSE3 support")
else()
    message(FATAL_ERROR "The target machine does not support SSE3. At least SSE3 is required")
endif()

if (MSVC)
    add_definitions(-DCONSTEXPR=const -DNOMINMAX -D_CRT_SECURE_NO_WARNINGS)
else()
    add_definitions(-DCONSTEXPR=constexpr)
endif()

IF(COMMAND cmake_policy)
  if (POLICY CMP0043)
    cmake_policy(SET CMP0043 NEW)
  endif()
ENDIF(COMMAND cmake_policy)

foreach(flag ${Vc_ARCHITECTURE_FLAGS})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${flag}")
endforeach()

add_definitions(-DEMBREE_STATIC_LIB=1)

add_definitions(-DRAPIDJSON_HAS_STDSTRING=1)
add_definitions(-DSTBI_NO_STDIO=1)
add_definitions(-DLODEPNG_NO_COMPILE_DISK=1)
add_definitions(-DUSE_IPV6=1)
add_library(tungsten_thirdparty STATIC
		src/thirdparty/lodepng/lodepng.cpp
		src/thirdparty/sobol/sobol.cpp
		src/thirdparty/stbi/stb_image.c
		src/thirdparty/miniz/miniz.c
		src/thirdparty/skylight/ArHosekSkyModel.cpp
		src/thirdparty/tribox/tribox.cpp)

if (CMAKE_COMPILER_IS_GNUCXX)
    set(CXX_WARNINGS "-Wall -Wextra -Wpointer-arith -Wcast-align -fstrict-aliasing -Wno-unused-local-typedefs -Wno-misleading-indentation -Wno-maybe-uninitialized -Wno-int-in-bool-context -Wno-implicit-fallthrough -Wno-class-memaccess")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_WARNINGS} -fvisibility-inlines-hidden")
endif()

include_directories(src/core src/thirdparty src)

INCLUDE_DIRECTORIES(
    "${CMAKE_CURRENT_SOURCE_DIR}/../openvdb/openvdb"
    "${CMAKE_CURRENT_BINARY_DIR}/../openvdb/openvdb"
    "${CMAKE_CURRENT_BINARY_DIR}/../openvdb/openvdb/openvdb"
    "${CMAKE_CURRENT_SOURCE_DIR}/../tbb/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../openexr/"
    "${CMAKE_CURRENT_BINARY_DIR}/../openexr/"
    "${CMAKE_CURRENT_SOURCE_DIR}/../openexr/Half"
    "${CMAKE_CURRENT_SOURCE_DIR}/../openexr/Iex"
    "${CMAKE_CURRENT_SOURCE_DIR}/../openexr/IlmImf"
    "${CMAKE_CURRENT_SOURCE_DIR}/../openexr/IlmThread"
    "${CMAKE_CURRENT_SOURCE_DIR}/../openexr/Imath"
    "${CMAKE_CURRENT_SOURCE_DIR}/../c-blosc/blosc/"
    "${CMAKE_CURRENT_SOURCE_DIR}/../libboost/boost_1_67_0/"
    "${CMAKE_CURRENT_SOURCE_DIR}/../eigen"
    "${CMAKE_CURRENT_SOURCE_DIR}/../eigen/unsupported"
    "${CMAKE_CURRENT_SOURCE_DIR}/../libjpeg/"
    "${CMAKE_CURRENT_BINARY_DIR}/../libjpeg/"
    "${CMAKE_CURRENT_SOURCE_DIR}/../embree/include/"
    "${CMAKE_CURRENT_BINARY_DIR}/../embree/include/"
    "${CMAKE_CURRENT_BINARY_DIR}/../embree/include/embree3/"
)

add_definitions(-DOPENEXR_AVAILABLE)
add_definitions(-DJPEG_AVAILABLE)
add_definitions(-DOPENVDB_AVAILABLE)
add_definitions(-DOPENVDB_3_ABI_COMPATIBLE)

file(GLOB_RECURSE Core_SOURCES "src/core/*.cpp")
add_library(tungsten STATIC ${Core_SOURCES})

file(GLOB_RECURSE Denoiser_SOURCES "src/denoiser/*.cpp")
add_library(denoiser STATIC ${Denoiser_SOURCES})

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(tungsten  PROPERTIES FOLDER external/tungsten)
  SET_TARGET_PROPERTIES(tungsten_thirdparty  PROPERTIES FOLDER external/tungsten)
  SET_TARGET_PROPERTIES(denoiser  PROPERTIES FOLDER external/tungsten)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)