cmake_minimum_required(VERSION 2.8)
project(SuiteSparse_AMD)

include_directories(Include)
include_directories(../SuiteSparse_config)
include_directories(../AMD/Include)
include_directories(../CAMD/Include)
include_directories(../COLAMD/Include)
include_directories(../CCOLAMD/Include)
include_directories(../metis-5.1.0/include)

option(WITH_CHOLMOD_CHECK "" ON)
option(WITH_CHOLMOD_CHOLESKY "" ON)
option(WITH_CHOLMOD_MATRIXOPS "" ON)
option(WITH_CHOLMOD_MODIFY "" ON)
option(WITH_CHOLMOD_SUPERNODAL "" ON)

option(WITH_CHOLMOD_CAMD "" ON)
option(WITH_CHOLMOD_PARTITION "" ON)

option(WITH_CHOLMOD_PRINT "" OFF)

add_definitions(-Wno-unused-parameter)

file(GLOB AMD_HEADERS "../AMD/Include/*.h")
file(GLOB CAMD_HEADERS "../CAMD/Include/*.h")
file(GLOB COLAMD_HEADERS "../COLAMD/Include/*.h")
file(GLOB CCOLAMD_HEADERS "../CCOLAMD/Include/*.h")
file(GLOB CHOLMOD_HEADERS "../Include/*.h")

file(GLOB CHOLMOD_SRCS
  "Core/*.c"
  "Check/*.c"
  "Cholesky/*.c"
  "MatrixOps/*.c"
  "Supernodal/*.c"
  "Modify/*.c"
  "Partition/*.c")
file(GLOB CHOLMOD_TEMPLATE_SRCS
  "Core/t*.c"
  "Cholesky/t*.c"
  "Check/t*.c"
  "MatrixOps/t*.c"
  "Supernodal/t*.c"
  "Modify/t*.c"
  "Partition/t*.c")
list(REMOVE_ITEM CHOLMOD_SRCS ${CHOLMOD_TEMPLATE_SRCS})

if(NOT WITH_CHOLMOD_PRINT)
  set_source_files_properties(${CHOLMOD_SRCS}
    PROPERTIES COMPILE_FLAGS "-DNPRINT")
endif()

if(NOT WITH_CHOLMOD_CHECK)
  file(GLOB CHOLMOD_CHECK_SRCS "Check/*.c")
  list(REMOVE_ITEM CHOLMOD_SRCS ${CHOLMOD_CHECK_SRCS})
endif()

if(NOT WITH_CHOLMOD_CHECK)
  file(GLOB CHOLMOD_CHOLESKY_SRCS "Cholesky/*.c")
  list(REMOVE_ITEM CHOLMOD_SRCS ${CHOLMOD_CHOLESKY_SRCS})
endif()

if(NOT WITH_CHOLMOD_CHECK)
  file(GLOB CHOLMOD_MATRIXOPS_SRCS "MatrixOps/*.c")
  list(REMOVE_ITEM CHOLMOD_SRCS ${CHOLMOD_MATRIXOPS_SRCS})
endif()

if(NOT WITH_CHOLMOD_CHECK)
  file(GLOB CHOLMOD_SUPERNODAL_SRCS "Supernodal/*.c")
  list(REMOVE_ITEM CHOLMOD_SRCS ${CHOLMOD_SUPERNODAL_SRCS})
endif()

if(NOT WITH_CHOLMOD_CHECK)
  file(GLOB CHOLMOD_MODIFY_SRCS "Modify/*.c")
  list(REMOVE_ITEM CHOLMOD_SRCS ${CHOLMOD_MODIFY_SRCS})
endif()

if(NOT WITH_CHOLMOD_CHECK)
  file(GLOB CHOLMOD_PARTITION_SRCS "Partition/*.c")
  list(REMOVE_ITEM CHOLMOD_SRCS ${CHOLMOD_PARTITION_SRCS})
endif()

set(LIBRARY_NAME "cholmod")

add_library(${LIBRARY_NAME} STATIC ${CHOLMOD_SRCS})
target_link_libraries(${LIBRARY_NAME} m)

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES FOLDER external/suitesparse)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)

set(LIBRARY_L_NAME "cholmod_l")

add_library(${LIBRARY_L_NAME} STATIC ${CHOLMOD_SRCS})
target_link_libraries(${LIBRARY_L_NAME} m)

target_compile_definitions (${LIBRARY_L_NAME} PRIVATE "-DDLONG")

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(${LIBRARY_L_NAME} PROPERTIES FOLDER external/suitesparse)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
