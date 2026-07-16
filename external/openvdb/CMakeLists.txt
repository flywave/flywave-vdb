set(USE_IMATH_HALF TRUE)

# OpenVDB 13 version (needed by version.h.in configure_file)
set(OpenVDB_MAJOR_VERSION 13)
set(OpenVDB_MINOR_VERSION 0)
set(OpenVDB_PATCH_VERSION 0)
set(OPENVDB_ABI_VERSION_NUMBER 13 CACHE STRING "OpenVDB ABI version" FORCE)

# Provide find_package config dirs for the vendored dependencies that
# openvdb 13's internal CMakeLists expects (Imath, TBB, Blosc, Boost).
set(Imath_DIR "${CMAKE_BINARY_DIR}/external/imath")
set(Blosc_DIR "${CMAKE_BINARY_DIR}/external/c-blosc/blosc")
set(TBB_DIR "${CMAKE_BINARY_DIR}/external/tbb-config-override")
set(Boost_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../libboost/boost_1_88_0/boost_unified")
set(Boost_NO_BOOST_CMAKE ON)

INCLUDE_DIRECTORIES(
    "${CMAKE_CURRENT_SOURCE_DIR}/openvdb"
    "${CMAKE_CURRENT_BINARY_DIR}/openvdb"
    "${CMAKE_CURRENT_SOURCE_DIR}/../tbb/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../imath/src"
    "${CMAKE_CURRENT_BINARY_DIR}/../imath"
    "${CMAKE_CURRENT_SOURCE_DIR}/../c-blosc/blosc"
    "${CMAKE_CURRENT_SOURCE_DIR}/../libboost/boost_1_88_0/boost_unified"
)

add_subdirectory(openvdb)
