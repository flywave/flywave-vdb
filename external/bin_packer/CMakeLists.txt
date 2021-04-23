
SET(bin_packer_SOURCE_FILES GuillotineBinPack.cpp Rect.cpp SkylineBinPack.cpp)
SET(bin_packer_HEADER_FILES GuillotineBinPack.h Rect.h SkylineBinPack.h)

ADD_LIBRARY(bin_packer STATIC
            ${bin_packer_HEADER_FILES}
            ${bin_packer_SOURCE_FILES}
          )

TARGET_INCLUDE_DIRECTORIES(bin_packer PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>)

SOURCE_GROUP("src" FILES ${bin_packer_HEADER_FILES})
SOURCE_GROUP("include" FILES ${bin_packer_HEADER_FILES})

SET_TARGET_PROPERTIES(bin_packer
    PROPERTIES PUBLIC_HEADER "${bin_packer_HEADER_FILES}")

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(bin_packer PROPERTIES FOLDER external)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
