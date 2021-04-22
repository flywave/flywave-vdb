# a simple way to detect that we are using CMAKE
add_definitions(-DUSING_CMAKE)

set(INTERNAL_LIBS ${CMAKE_SOURCE_DIR}/internal-complibs)

INCLUDE_DIRECTORIES(BEFORE ${CMAKE_CURRENT_BINARY_DIR}/)

set(BLOSC_INCLUDE_DIRS ${BLOSC_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

set(BLOSC_INCLUDE_DIRS ${BLOSC_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../../lz4)

set(BLOSC_INCLUDE_DIRS ${BLOSC_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../../snappy)

IF (WIN32)
  set(BLOSC_INCLUDE_DIRS ${BLOSC_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../../snappy//win32)
ENDIF()

IF(APPLE)
  set(BLOSC_INCLUDE_DIRS ${BLOSC_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../../snappy//mac)
ENDIF()

IF (CMAKE_SYSTEM_NAME MATCHES "Linux")
  set(BLOSC_INCLUDE_DIRS ${BLOSC_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../../snappy//linux)
ENDIF()

set(BLOSC_INCLUDE_DIRS ${BLOSC_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../../zlib)

set(BLOSC_INCLUDE_DIRS ${BLOSC_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../../zstd/lib ${CMAKE_CURRENT_SOURCE_DIR}/../zstd/lib/common)

include_directories(${BLOSC_INCLUDE_DIRS})

# library sources
set(SOURCES blosc.c blosclz.c shuffle.c)
# library install directory
set(lib_dir lib${LIB_SUFFIX})
set(version_string ${BLOSC_VERSION_MAJOR}.${BLOSC_VERSION_MINOR}.${BLOSC_VERSION_PATCH})

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
if(WIN32)
    # try to use the system library
    find_package(Threads)
    if(NOT Threads_FOUND)
        message(STATUS "using the internal pthread library for win32 systems.")
        set(SOURCES ${SOURCES} win32/pthread.c)
    else(NOT Threads_FOUND)
        set(LIBS ${LIBS} ${CMAKE_THREAD_LIBS_INIT})
    endif(NOT Threads_FOUND)
else(WIN32)
    find_package(Threads REQUIRED)
    set(LIBS ${LIBS} ${CMAKE_THREAD_LIBS_INIT})
endif(WIN32)

# targets
add_library(blosc STATIC ${SOURCES})
set_target_properties(blosc PROPERTIES OUTPUT_NAME blosc)
if (MSVC)
    set_target_properties(blosc PROPERTIES PREFIX lib)
endif()
target_link_libraries(blosc ${LIBS})

set(HEADER_FILES blosc.h)

SOURCE_GROUP("src" FILES ${SOURCES})
SOURCE_GROUP("include" FILES ${HEADER_FILES})

SET_TARGET_PROPERTIES(blosc
    PROPERTIES PUBLIC_HEADER "${headers}")

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(blosc PROPERTIES FOLDER external)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
