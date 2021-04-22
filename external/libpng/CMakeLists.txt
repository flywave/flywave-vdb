set(CMAKE_C_FLAGS "-Wno-shift-negative-value")
if(UNIX OR APPLE)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -Wall")
endif()

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} include "${CMAKE_CURRENT_SOURCE_DIR}/../zlib" )

SET(png_HEADER_FILES
  png.h
  pngconf.h
)

SET(png_SOURCE_FILES
  png.c
  pngerror.c
  pngget.c
  pngmem.c
  pngpread.c
  pngread.c
  pngrio.c
  pngrtran.c
  pngrutil.c
  pngset.c
  pngtrans.c
  pngwio.c
  pngwrite.c
  pngwtran.c
  pngwutil.c
)

add_library (png STATIC
	${png_HEADER_FILES}
	${png_SOURCE_FILES}
)

SET(png_DEPENDENCIES zlib)

SET(png_LINK_LIBRARIES zlib)

SOURCE_GROUP("src" FILES ${png_SOURCE_FILES})
SOURCE_GROUP("include" FILES ${png_HEADER_FILES})

target_link_libraries(png zlib)

SET_TARGET_PROPERTIES(png
    PROPERTIES PUBLIC_HEADER "${headers}")

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(png PROPERTIES FOLDER external)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
