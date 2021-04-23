
cmake_minimum_required(VERSION 2.8)
project(SuiteSparse_COLAMD)

include_directories(Include)
include_directories(../SuiteSparse_config)

add_definitions(-Wno-unused-parameter)

file(GLOB COLAMD_SRCS "Source/*.c")

set(LIBRARY_NAME "colamd")

add_library(${LIBRARY_NAME} STATIC ${COLAMD_SRCS})
target_link_libraries(${LIBRARY_NAME} m)

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES FOLDER external/suitesparse)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)

set(LIBRARY_L_NAME "colamd_l")

add_library(${LIBRARY_L_NAME} STATIC ${COLAMD_SRCS})
target_link_libraries(${LIBRARY_L_NAME} m)

target_compile_definitions (${LIBRARY_L_NAME} PRIVATE "-DDLONG")

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(${LIBRARY_L_NAME} PROPERTIES FOLDER external/suitesparse)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
