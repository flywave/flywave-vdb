
set(lzma_HEADER_FILES
${CMAKE_CURRENT_SOURCE_DIR}/api/lzma/base.h
${CMAKE_CURRENT_SOURCE_DIR}/api/lzma/bcj.h
${CMAKE_CURRENT_SOURCE_DIR}/api/lzma/block.h
${CMAKE_CURRENT_SOURCE_DIR}/api/lzma/check.h
${CMAKE_CURRENT_SOURCE_DIR}/api/lzma/container.h
${CMAKE_CURRENT_SOURCE_DIR}/api/lzma/delta.h
${CMAKE_CURRENT_SOURCE_DIR}/api/lzma/filter.h
${CMAKE_CURRENT_SOURCE_DIR}/api/lzma/hardware.h
${CMAKE_CURRENT_SOURCE_DIR}/api/lzma/index.h
${CMAKE_CURRENT_SOURCE_DIR}/api/lzma/index_hash.h
${CMAKE_CURRENT_SOURCE_DIR}/api/lzma/lzma12.h
${CMAKE_CURRENT_SOURCE_DIR}/api/lzma/stream_flags.h
${CMAKE_CURRENT_SOURCE_DIR}/api/lzma/version.h
${CMAKE_CURRENT_SOURCE_DIR}/api/lzma/vli.h
${CMAKE_CURRENT_SOURCE_DIR}/api/lzma.h
${CMAKE_CURRENT_SOURCE_DIR}/check/check.h
${CMAKE_CURRENT_SOURCE_DIR}/check/crc32_table_be.h
${CMAKE_CURRENT_SOURCE_DIR}/check/crc32_table_le.h
${CMAKE_CURRENT_SOURCE_DIR}/check/crc64_table_be.h
${CMAKE_CURRENT_SOURCE_DIR}/check/crc64_table_le.h
${CMAKE_CURRENT_SOURCE_DIR}/check/crc_macros.h
${CMAKE_CURRENT_SOURCE_DIR}/common/alone_decoder.h
${CMAKE_CURRENT_SOURCE_DIR}/common/block_decoder.h
${CMAKE_CURRENT_SOURCE_DIR}/common/block_encoder.h
${CMAKE_CURRENT_SOURCE_DIR}/common/common.h
${CMAKE_CURRENT_SOURCE_DIR}/common/easy_preset.h
${CMAKE_CURRENT_SOURCE_DIR}/common/filter_common.h
${CMAKE_CURRENT_SOURCE_DIR}/common/filter_decoder.h
${CMAKE_CURRENT_SOURCE_DIR}/common/filter_encoder.h
${CMAKE_CURRENT_SOURCE_DIR}/common/index.h
${CMAKE_CURRENT_SOURCE_DIR}/common/index_encoder.h
${CMAKE_CURRENT_SOURCE_DIR}/common/stream_decoder.h
${CMAKE_CURRENT_SOURCE_DIR}/common/stream_flags_common.h
${CMAKE_CURRENT_SOURCE_DIR}/delta/delta_common.h
${CMAKE_CURRENT_SOURCE_DIR}/delta/delta_decoder.h
${CMAKE_CURRENT_SOURCE_DIR}/delta/delta_encoder.h
${CMAKE_CURRENT_SOURCE_DIR}/delta/delta_private.h
${CMAKE_CURRENT_SOURCE_DIR}/lz/lz_decoder.h
${CMAKE_CURRENT_SOURCE_DIR}/lz/lz_encoder.h
${CMAKE_CURRENT_SOURCE_DIR}/lz/lz_encoder_hash.h
${CMAKE_CURRENT_SOURCE_DIR}/lz/lz_encoder_hash_table.h
${CMAKE_CURRENT_SOURCE_DIR}/lzma/fastpos.h
${CMAKE_CURRENT_SOURCE_DIR}/lzma/lzma2_decoder.h
${CMAKE_CURRENT_SOURCE_DIR}/lzma/lzma2_encoder.h
${CMAKE_CURRENT_SOURCE_DIR}/lzma/lzma_common.h
${CMAKE_CURRENT_SOURCE_DIR}/lzma/lzma_decoder.h
${CMAKE_CURRENT_SOURCE_DIR}/lzma/lzma_encoder.h
${CMAKE_CURRENT_SOURCE_DIR}/lzma/lzma_encoder_private.h
${CMAKE_CURRENT_SOURCE_DIR}/rangecoder/price.h
${CMAKE_CURRENT_SOURCE_DIR}/rangecoder/range_common.h
${CMAKE_CURRENT_SOURCE_DIR}/rangecoder/range_decoder.h
${CMAKE_CURRENT_SOURCE_DIR}/rangecoder/range_encoder.h
${CMAKE_CURRENT_SOURCE_DIR}/simple/simple_coder.h
${CMAKE_CURRENT_SOURCE_DIR}/simple/simple_decoder.h
${CMAKE_CURRENT_SOURCE_DIR}/simple/simple_encoder.h
${CMAKE_CURRENT_SOURCE_DIR}/simple/simple_private.h
)
set(lzma_SOURCE_FILES
${CMAKE_CURRENT_SOURCE_DIR}/check/check.c
${CMAKE_CURRENT_SOURCE_DIR}/check/crc32_fast.c
#${CMAKE_CURRENT_SOURCE_DIR}/check/crc32_small.c
${CMAKE_CURRENT_SOURCE_DIR}/check/crc32_table.c
#${CMAKE_CURRENT_SOURCE_DIR}/check/crc32_tablegen.c
${CMAKE_CURRENT_SOURCE_DIR}/check/crc64_fast.c
#${CMAKE_CURRENT_SOURCE_DIR}/check/crc64_small.c
${CMAKE_CURRENT_SOURCE_DIR}/check/crc64_table.c
#${CMAKE_CURRENT_SOURCE_DIR}/check/crc64_tablegen.c
${CMAKE_CURRENT_SOURCE_DIR}/check/sha256.c
${CMAKE_CURRENT_SOURCE_DIR}/common/alone_decoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/alone_encoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/auto_decoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/block_buffer_decoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/block_buffer_encoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/block_decoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/block_encoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/block_header_decoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/block_header_encoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/block_util.c
${CMAKE_CURRENT_SOURCE_DIR}/common/common.c
${CMAKE_CURRENT_SOURCE_DIR}/common/easy_buffer_encoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/easy_decoder_memusage.c
${CMAKE_CURRENT_SOURCE_DIR}/common/easy_encoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/easy_encoder_memusage.c
${CMAKE_CURRENT_SOURCE_DIR}/common/easy_preset.c
${CMAKE_CURRENT_SOURCE_DIR}/common/filter_buffer_decoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/filter_buffer_encoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/filter_common.c
${CMAKE_CURRENT_SOURCE_DIR}/common/filter_decoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/filter_encoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/filter_flags_decoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/filter_flags_encoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/hardware_physmem.c
${CMAKE_CURRENT_SOURCE_DIR}/common/index.c
${CMAKE_CURRENT_SOURCE_DIR}/common/index_decoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/index_encoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/index_hash.c
${CMAKE_CURRENT_SOURCE_DIR}/common/stream_buffer_decoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/stream_buffer_encoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/stream_decoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/stream_encoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/stream_flags_common.c
${CMAKE_CURRENT_SOURCE_DIR}/common/stream_flags_decoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/stream_flags_encoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/vli_decoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/vli_encoder.c
${CMAKE_CURRENT_SOURCE_DIR}/common/vli_size.c
${CMAKE_CURRENT_SOURCE_DIR}/delta/delta_common.c
${CMAKE_CURRENT_SOURCE_DIR}/delta/delta_decoder.c
${CMAKE_CURRENT_SOURCE_DIR}/delta/delta_encoder.c
${CMAKE_CURRENT_SOURCE_DIR}/lz/lz_decoder.c
${CMAKE_CURRENT_SOURCE_DIR}/lz/lz_encoder.c
${CMAKE_CURRENT_SOURCE_DIR}/lz/lz_encoder_mf.c
${CMAKE_CURRENT_SOURCE_DIR}/lzma/fastpos_table.c
#${CMAKE_CURRENT_SOURCE_DIR}/lzma/fastpos_tablegen.c
${CMAKE_CURRENT_SOURCE_DIR}/lzma/lzma2_decoder.c
${CMAKE_CURRENT_SOURCE_DIR}/lzma/lzma2_encoder.c
${CMAKE_CURRENT_SOURCE_DIR}/lzma/lzma_decoder.c
${CMAKE_CURRENT_SOURCE_DIR}/lzma/lzma_encoder.c
${CMAKE_CURRENT_SOURCE_DIR}/lzma/lzma_encoder_optimum_fast.c
${CMAKE_CURRENT_SOURCE_DIR}/lzma/lzma_encoder_optimum_normal.c
${CMAKE_CURRENT_SOURCE_DIR}/lzma/lzma_encoder_presets.c
${CMAKE_CURRENT_SOURCE_DIR}/rangecoder/price_table.c
#${CMAKE_CURRENT_SOURCE_DIR}/rangecoder/price_tablegen.c
${CMAKE_CURRENT_SOURCE_DIR}/simple/arm.c
${CMAKE_CURRENT_SOURCE_DIR}/simple/armthumb.c
${CMAKE_CURRENT_SOURCE_DIR}/simple/ia64.c
${CMAKE_CURRENT_SOURCE_DIR}/simple/powerpc.c
${CMAKE_CURRENT_SOURCE_DIR}/simple/simple_coder.c
${CMAKE_CURRENT_SOURCE_DIR}/simple/simple_decoder.c
${CMAKE_CURRENT_SOURCE_DIR}/simple/simple_encoder.c
${CMAKE_CURRENT_SOURCE_DIR}/simple/sparc.c
${CMAKE_CURRENT_SOURCE_DIR}/simple/x86.c
${CMAKE_CURRENT_SOURCE_DIR}/xzcommon/tuklib_physmem.c
)

#add_definitions(-DLIB_CONFIGURE_EXPORT)

if(MSVC)
  add_definitions(-D_CRT_SECURE_NO_DEPRECATE -DHAVE_CONFIG_H)
endif(MSVC)

add_definitions(-DHAVE_CONFIG_H)

set(lzma_self_INCLUDE_DIRS
${CMAKE_CURRENT_SOURCE_DIR}/api
${CMAKE_CURRENT_SOURCE_DIR}/common
${CMAKE_CURRENT_SOURCE_DIR}/check
${CMAKE_CURRENT_SOURCE_DIR}/lz
${CMAKE_CURRENT_SOURCE_DIR}/rangecoder
${CMAKE_CURRENT_SOURCE_DIR}/lzma
${CMAKE_CURRENT_SOURCE_DIR}/delta
${CMAKE_CURRENT_SOURCE_DIR}/simple
xzcommon
)

set(lzma_INCLUDE_DIRS
${CMAKE_CURRENT_SOURCE_DIR}/api
)

include_directories(${lzma_self_INCLUDE_DIRS})

if (WIN32)
  include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/win32)
  set(lzma_ex_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/win32)
ELSEIF (APPLE)
  include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/mac)
  set(lzma_ex_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/mac)
ELSEIF (UNIX)
 SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-parentheses-equality ")
  include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/linux)
  set(lzma_ex_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/linux)
endif()


ADD_LIBRARY(lzma STATIC
            ${lzma_HEADER_FILES}
            ${lzma_SOURCE_FILES}
          )

TARGET_INCLUDE_DIRECTORIES(lzma PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>)

SOURCE_GROUP("src" FILES ${lzma_SOURCE_FILES})
SOURCE_GROUP("include" FILES ${lzma_HEADER_FILES})

SET_TARGET_PROPERTIES(lzma
                    PROPERTIES PUBLIC_HEADER "${headers}")

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(lzma PROPERTIES FOLDER external)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
