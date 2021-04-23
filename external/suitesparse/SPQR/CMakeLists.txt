
cmake_minimum_required(VERSION 2.8)
project(SuiteSparse_SPQR)

include_directories(Include)
include_directories(../SuiteSparse_config)
include_directories(../CHOLMOD/Include)

add_definitions(-Wno-unused-parameter)

file(GLOB SPQR_SRCS "Source/*.cpp")

set(LIBRARY_NAME "spqr")

add_library(${LIBRARY_NAME} STATIC ${SPQR_SRCS})
target_link_libraries(${LIBRARY_NAME} m)

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES FOLDER external/suitesparse)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)