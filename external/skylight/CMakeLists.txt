
IF(UNIX)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-float-conversion ")
ENDIF()

set(skylight_SOURCE_FILES ArHosekSkyModel.c)
set(skylight_HEADER_FILES ArHosekSkyModel.h ArHosekSkyModelData_CIEXYZ.h ArHosekSkyModelData_RGB.h ArHosekSkyModelData_Spectral.h)

ADD_LIBRARY(skylight STATIC
            ${skylight_HEADER_FILES}
            ${skylight_SOURCE_FILES}
          )

TARGET_INCLUDE_DIRECTORIES(skylight PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>)

SOURCE_GROUP("src" FILES ${skylight_SOURCE_FILES})
SOURCE_GROUP("include" FILES ${skylight_HEADER_FILES})

SET_TARGET_PROPERTIES(skylight
                    PROPERTIES PUBLIC_HEADER "${headers}")

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(skylight PROPERTIES FOLDER external)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
