
INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/include")

SET(lerc_SOURCE_FILES
    src/LercLib/BitMask.cpp
    src/LercLib/BitStuffer2.cpp
    src/LercLib/Huffman.cpp
    src/LercLib/Lerc_c_api_impl.cpp
    src/LercLib/Lerc.cpp
    src/LercLib/Lerc2.cpp
    src/LercLib/RLE.cpp
    src/LercLib/Lerc1Decode/CntZImage.cpp
    src/LercLib/Lerc1Decode/BitStuffer.cpp
)
SET(lerc_HEADER_FILES include/Lerc_c_api.h include/Lerc_types.h src/LercLib/Lerc.h)

IF(UNIX)
 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -fPIC")
ENDIF()

ADD_LIBRARY(lerc STATIC
		          ${lerc_HEADER_FILES}
		          ${lerc_SOURCE_FILES}
		        )

TARGET_INCLUDE_DIRECTORIES(lerc PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>)

SOURCE_GROUP("src" FILES ${lerc_SOURCE_FILES})
SOURCE_GROUP("include" FILES ${lerc_HEADER_FILES})

SET_TARGET_PROPERTIES(lerc
		                  PROPERTIES PUBLIC_HEADER "${headers}")

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(lerc PROPERTIES FOLDER external)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
