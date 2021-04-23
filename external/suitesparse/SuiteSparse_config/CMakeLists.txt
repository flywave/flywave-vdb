cmake_minimum_required(VERSION 2.8)
project(SuiteSparse_config)

set(LIBRARY_NAME SuiteSparse_config)
add_library(${LIBRARY_NAME} STATIC SuiteSparse_config.c)
target_link_libraries(${LIBRARY_NAME})

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES FOLDER external/suitesparse)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
