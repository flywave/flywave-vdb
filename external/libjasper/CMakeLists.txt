# ----------------------------------------------------------------------------
#  CMake file for libjasper. See root CMakeLists.txt
#
# ----------------------------------------------------------------------------
project(jasper)

IF(UNIX)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-shadow -Wno-self-assign -Wno-float-conversion -Wno-unused-function -Wno-implicit-function-declaration -Wno-unused-variable -Wno-unused-const-variable -Wno-switch -Wno-unknown-pragmas -Wno-enum-compare -Wno-unused-result  -Wno-tautological-compare -Wno-attributes -Wno-sign-compare -Wno-deprecated-declarations")
IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-but-set-variable -Wno-tautological-compare -Wno-unused-const-variable")
ENDIF()
ENDIF()

add_definitions(-DEXCLUDE_MIF_SUPPORT -DEXCLUDE_PNM_SUPPORT -DEXCLUDE_BMP_SUPPORT -DEXCLUDE_RAS_SUPPORT  -DEXCLUDE_JPG_SUPPORT -DEXCLUDE_PGX_SUPPORT)

INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}")

file(GLOB lib_srcs *.c)
file(GLOB lib_hdrs *.h)
file(GLOB lib_ext_hdrs jasper/*.h)

# ----------------------------------------------------------------------------------
#         Define the library target:
# ----------------------------------------------------------------------------------

add_library(jasper STATIC ${lib_srcs} ${lib_hdrs} ${lib_ext_hdrs})

if(WIN32 AND NOT MINGW)
  add_definitions(-DJAS_WIN_MSVC_BUILD)
endif(WIN32 AND NOT MINGW)

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(jasper PROPERTIES FOLDER external)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
