cmake_minimum_required(VERSION 2.8)
project(GKlib)

get_filename_component(abs "." ABSOLUTE)
set(GKLIB_PATH ${abs})
unset(abs)
include(GKlibSystem.cmake)

include_directories(".")
add_library(GKlib STATIC ${GKlib_sources})
if(UNIX)
  target_link_libraries(GKlib m)
endif(UNIX)

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(GKlib PROPERTIES FOLDER external/suitesparse)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
