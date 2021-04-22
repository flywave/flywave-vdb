# ----------------------------------------------------------------------------
#  CMake file for openexr
#
# ----------------------------------------------------------------------------

project(openexr CXX)

macro(ocv_list_filterout lst regex)
  foreach(item ${${lst}})
    if(item MATCHES "${regex}")
      list(REMOVE_ITEM ${lst} "${item}")
    endif()
  endforeach()
endmacro()

IF(UNIX)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-shadow -Wno-self-assign -Wno-float-conversion -Wno-unused-function -Wno-implicit-function-declaration -Wno-unused-variable -Wno-unused-const-variable -Wno-switch -Wno-unknown-pragmas -Wno-enum-compare -Wno-unused-result  -Wno-tautological-compare -Wno-attributes -Wno-sign-compare -Wno-deprecated-declarations")
IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-but-set-variable -Wno-tautological-compare -Wno-unused-const-variable")
ENDIF()
ENDIF()

if(UNIX)
  set(HAVE_PTHREAD 1)
  if(APPLE)
    set(HAVE_POSIX_SEMAPHORES 0)  # Unnamed semaphores are not supported: https://github.com/opencv/opencv/issues/9361
  else()
    include(CheckIncludeFile)
    check_include_file(semaphore.h HAVE_POSIX_SEMAPHORES)
  endif()
endif()

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/IlmBaseConfig.h.cmakein"
               "${CMAKE_CURRENT_BINARY_DIR}/IlmBaseConfig.h" @ONLY)
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/OpenEXRConfig.h.cmakein"
               "${CMAKE_CURRENT_BINARY_DIR}/OpenEXRConfig.h" @ONLY)

set(OPENEXR_INCLUDE_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/Half"
                          "${CMAKE_CURRENT_SOURCE_DIR}/Iex"
                          "${CMAKE_CURRENT_SOURCE_DIR}/IexMath"
                          "${CMAKE_CURRENT_SOURCE_DIR}/IlmThread"
                          "${CMAKE_CURRENT_SOURCE_DIR}/Imath"
                          "${CMAKE_CURRENT_SOURCE_DIR}/IlmImf"
                          "${CMAKE_CURRENT_SOURCE_DIR}/IlmImfUtil"
                          "${CMAKE_CURRENT_BINARY_DIR}/")

INCLUDE_DIRECTORIES("${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/../zlib"  ${OPENEXR_INCLUDE_PATHS})

file(GLOB lib_srcs Half/half.cpp Iex/*.cpp IexMath/*.cpp IlmImfUtil/*.cpp IlmThread/*.cpp Imath/*.cpp IlmImf/*.cpp)
file(GLOB lib_hdrs Half/*.h Iex/Iex*.h IexMath/*.h IlmThread/IlmThread*.h IlmImfUtil/*.h Imath/Imath*.h IlmImf/*.h)
list(APPEND lib_hdrs "${CMAKE_CURRENT_BINARY_DIR}/IlmBaseConfig.h" "${CMAKE_CURRENT_BINARY_DIR}/OpenEXRConfig.h")

ocv_list_filterout(lib_srcs IlmImf/b44ExpLogTable.cpp)
ocv_list_filterout(lib_srcs IlmImf/dwaLookups.cpp)

if(WIN32)
  ocv_list_filterout(lib_srcs Posix.*cpp)
else()
  ocv_list_filterout(lib_srcs Win32.cpp)
endif()

source_group("Include" FILES ${lib_hdrs} )
source_group("Src" FILES ${lib_srcs})

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Qrestrict")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /Qrestrict")
endif()

add_library(IlmImf STATIC ${lib_hdrs} ${lib_srcs})
target_link_libraries(IlmImf ${ZLIB_LIBRARIES})

set(OPENEXR_INCLUDE_PATHS ${OPENEXR_INCLUDE_PATHS} PARENT_SCOPE)
set(OPENEXR_VERSION "1.7.1" PARENT_SCOPE)

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(IlmImf PROPERTIES FOLDER external)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
