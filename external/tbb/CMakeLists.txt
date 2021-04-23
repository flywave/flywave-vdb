cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project (tbb CXX)

include(CheckCXXCompilerFlag)
include(CheckCXXSourceRuns)

INCLUDE_DIRECTORIES(
  "${CMAKE_CURRENT_SOURCE_DIR}/../tbb/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/../c-blosc"
)

include_directories(include src src/rml/include ${CMAKE_CURRENT_BINARY_DIR})

if(APPLE)
  set(CMAKE_MACOSX_RPATH ON)
endif()

file(GLOB tbb_src "${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/old/*.cpp")
list(REMOVE_ITEM tbb_src ${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/tbb_bind.cpp)
list(APPEND tbb_src ${CMAKE_CURRENT_SOURCE_DIR}/src/rml/client/rml_tbb.cpp)
file(GLOB to_remove "${CMAKE_CURRENT_SOURCE_DIR}/src/old/test*.cpp")
list(REMOVE_ITEM tbb_src ${to_remove})

set(tbbmalloc_static_src
  src/tbbmalloc/backend.cpp
  src/tbbmalloc/large_objects.cpp
  src/tbbmalloc/backref.cpp
  src/tbbmalloc/tbbmalloc.cpp
  src/tbbmalloc/frontend.cpp
  src/tbb/itt_notify.cpp)

set(tbbmalloc_src ${tbbmalloc_static_src})

set(tbbmalloc_proxy_src
  src/tbbmalloc/proxy.cpp
  src/tbbmalloc/tbb_function_replacement.cpp)

add_library (tbb_interface INTERFACE)
add_definitions(-DTBB_SUPPRESS_DEPRECATED_MESSAGES=1)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "(i386|x86_64)")
  if (NOT APPLE AND NOT MINGW)
    target_compile_definitions(tbb_interface INTERFACE DO_ITT_NOTIFY)
  endif()
endif()

if (APPLE)
  set(CMAKE_C_ARCHIVE_CREATE   "<CMAKE_AR> Scr <TARGET> <LINK_FLAGS> <OBJECTS>")
  set(CMAKE_CXX_ARCHIVE_CREATE "<CMAKE_AR> Scr <TARGET> <LINK_FLAGS> <OBJECTS>")
  set(CMAKE_C_ARCHIVE_FINISH   "<CMAKE_RANLIB> -no_warning_for_no_symbols -c <TARGET>")
  set(CMAKE_CXX_ARCHIVE_FINISH "<CMAKE_RANLIB> -no_warning_for_no_symbols -c <TARGET>")
endif()

macro(CHECK_CXX_COMPILER_AND_LINKER_FLAGS _RESULT _CXX_FLAGS _LINKER_FLAGS)
  set(CMAKE_REQUIRED_FLAGS ${_CXX_FLAGS})
  set(CMAKE_REQUIRED_LIBRARIES ${_LINKER_FLAGS})
  set(CMAKE_REQUIRED_QUIET TRUE)
  check_cxx_source_runs("#include <iostream>\nint main(int argc, char **argv) { std::cout << \"test\"; return 0; }" ${_RESULT})
  set(CMAKE_REQUIRED_FLAGS "")
  set(CMAKE_REQUIRED_LIBRARIES "")
endmacro()

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  if (CMAKE_CXX_FLAGS MATCHES "-stdlib=libc\\+\\+")
    message(STATUS "TBB: using libc++.")
  else()
    CHECK_CXX_COMPILER_AND_LINKER_FLAGS(HAS_LIBCPP "-stdlib=libc++" "-stdlib=libc++")
    if (HAS_LIBCPP)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++ -D_LIBCPP_VERSION")
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libc++")
      set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -stdlib=libc++")
      message(STATUS "TBB: using libc++.")
    else()
      message(STATUS "TBB: NOT using libc++.")
    endif()
  endif()
endif()

set (CMAKE_CXX_STANDARD 11)

if (UNIX)
  target_compile_definitions(tbb_interface INTERFACE USE_PTHREAD)

  check_cxx_compiler_flag ("-mrtm -Werror" SUPPORTS_MRTM)
  if (SUPPORTS_MRTM)
    target_compile_options(tbb_interface INTERFACE "-mrtm")
  endif ()

elseif(WIN32)
  target_compile_definitions(tbb_interface INTERFACE USE_WINTHREAD _WIN32_WINNT=0x0600)
  if (MSVC)
    enable_language(ASM_MASM)
    target_compile_options(tbb_interface INTERFACE /GS- /Zc:wchar_t /Zc:forScope)
    check_cxx_compiler_flag ("/volatile:iso" SUPPORTS_VOLATILE_FLAG)
    if (SUPPORTS_VOLATILE_FLAG)
      target_compile_options(tbb_interface INTERFACE /volatile:iso)
    endif ()
    target_compile_options(tbb_interface INTERFACE $<$<COMPILE_LANGUAGE:CXX>:/wd4267 /wd4800 /wd4146 /wd4244 /wd4577 /wd4018>)
    if (NOT CMAKE_SIZEOF_VOID_P)
       message(FATAL_ERROR "'CMAKE_SIZEOF_VOID_P' is undefined. Please delete your build directory and rerun CMake again!")
    endif()

    if (CMAKE_SIZEOF_VOID_P EQUAL 8)
      list(APPEND tbb_src src/tbb/intel64-masm/atomic_support.asm
        src/tbb/intel64-masm/itsx.asm src/tbb/intel64-masm/intel64_misc.asm)
      list(APPEND tbbmalloc_src src/tbb/intel64-masm/atomic_support.asm)
      set(CMAKE_ASM_MASM_FLAGS "/DEM64T=1 ${CMAKE_ASM_MASM_FLAGS}")
    else()
      list(APPEND tbb_src src/tbb/ia32-masm/atomic_support.asm
        src/tbb/ia32-masm/itsx.asm src/tbb/ia32-masm/lock_byte.asm)

      set(CMAKE_ASM_MASM_FLAGS "/safeseh ${CMAKE_ASM_MASM_FLAGS}")
    endif()
  elseif (MINGW)
    target_compile_options(tbb_interface INTERFACE "-mthreads")
  endif ()
endif()

if (MSVC)
  set(ENABLE_RTTI "/EHsc /GR ")
  set(DISABLE_RTTI "/EHs- /GR- ")
elseif (UNIX)
  set(ENABLE_RTTI "-frtti -fexceptions ")
  set(DISABLE_RTTI "-fno-rtti -fno-exceptions ")
endif ()

if (NOT TBB_USE_GLIBCXX_VERSION AND UNIX AND NOT APPLE)
  if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    string(REPLACE "." "0" TBB_USE_GLIBCXX_VERSION ${CMAKE_CXX_COMPILER_VERSION})
  endif()
endif()

if (TBB_USE_GLIBCXX_VERSION)
  target_compile_definitions(tbb_interface INTERFACE TBB_USE_GLIBCXX_VERSION=${TBB_USE_GLIBCXX_VERSION})
endif()

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
   check_cxx_compiler_flag ("-flifetime-dse=1" SUPPORTS_FLIFETIME)
   if (SUPPORTS_FLIFETIME)
     target_compile_options(tbb_interface INTERFACE -flifetime-dse=1)
   endif()
endif()

if (APPLE)
  set (ARCH_PREFIX "mac")
elseif(WIN32)
  set (ARCH_PREFIX "win")
else()
  set (ARCH_PREFIX "lin")
endif()

if (CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(ARCH_PREFIX "${ARCH_PREFIX}64")
else()
  set(ARCH_PREFIX "${ARCH_PREFIX}32")
endif()

if (MINGW)
  set (ARCH_PREFIX "${ARCH_PREFIX}-gcc")
  execute_process (COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/lin32-tbb-export.def ${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/win32-gcc-tbb-export.def)
endif()

if (MSVC)
  add_custom_command(OUTPUT tbb.def
    COMMAND ${CMAKE_CXX_COMPILER} /TC /EP ${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/${ARCH_PREFIX}-tbb-export.def  -I ${CMAKE_CURRENT_SOURCE_DIR}/include > tbb.def
    MAIN_DEPENDENCY ${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/${ARCH_PREFIX}-tbb-export.def
    COMMENT "Preprocessing tbb.def"
  )

  add_custom_command(OUTPUT tbbmalloc.def
    COMMAND ${CMAKE_CXX_COMPILER} /TC /EP ${CMAKE_CURRENT_SOURCE_DIR}/src/tbbmalloc/${ARCH_PREFIX}-tbbmalloc-export.def  -I ${CMAKE_CURRENT_SOURCE_DIR}/include >   tbbmalloc.def
    MAIN_DEPENDENCY ${CMAKE_CURRENT_SOURCE_DIR}/src/tbbmalloc/${ARCH_PREFIX}-tbbmalloc-export.def
    COMMENT "Preprocessing tbbmalloc.def"
  )
else()
  add_custom_command(OUTPUT tbb.def
    COMMAND ${CMAKE_CXX_COMPILER} -xc++ -std=c++11 -E ${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/${ARCH_PREFIX}-tbb-export.def  -I ${CMAKE_CURRENT_SOURCE_DIR}/include -o tbb.def
    MAIN_DEPENDENCY ${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/${ARCH_PREFIX}-tbb-export.def
    COMMENT "Preprocessing tbb.def"
  )

  add_custom_command(OUTPUT tbbmalloc.def
    COMMAND ${CMAKE_CXX_COMPILER} -xc++ -std=c++11 -E ${CMAKE_CURRENT_SOURCE_DIR}/src/tbbmalloc/${ARCH_PREFIX}-tbbmalloc-export.def  -I ${CMAKE_CURRENT_SOURCE_DIR}/include -o   tbbmalloc.def
    MAIN_DEPENDENCY ${CMAKE_CURRENT_SOURCE_DIR}/src/tbbmalloc/${ARCH_PREFIX}-tbbmalloc-export.def
    COMMENT "Preprocessing tbbmalloc.def"
  )
endif()

add_custom_target(tbb_def_files DEPENDS tbb.def tbbmalloc.def)

add_library(tbb STATIC ${tbb_src})
target_link_libraries(tbb PRIVATE tbb_interface)
target_include_directories(tbb INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" "$<INSTALL_INTERFACE:${TBB_INSTALL_INCLUDE_DIR}>")
set_property(TARGET tbb APPEND_STRING PROPERTY COMPILE_FLAGS ${ENABLE_RTTI})

target_compile_definitions(tbb
    PRIVATE
      -D__TBB_BUILD=1
      -D__TBB_DYNAMIC_LOAD_ENABLED=0
      -D__TBB_SOURCE_DIRECTLY_INCLUDED=1)

if (MSVC)
  target_compile_definitions(tbb
      PUBLIC -D__TBB_NO_IMPLICIT_LINKAGE=1
      PRIVATE -D_CRT_SECURE_NO_WARNINGS)
endif()

if (UNIX AND NOT APPLE)
  target_link_libraries(tbb PUBLIC pthread dl)
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/governor.cpp COMPILE_FLAGS "-Wno-missing-field-initializers ")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/itt_notify.cpp COMPILE_FLAGS "-Wno-varargs ")
elseif(MSVC)
  set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/scheduler.cpp COMPILE_FLAGS "/wd4458 ")
endif()

add_library(tbbmalloc STATIC ${tbbmalloc_static_src})
target_link_libraries(tbbmalloc PRIVATE tbb_interface)
set_property(TARGET tbbmalloc APPEND PROPERTY COMPILE_DEFINITIONS "__TBBMALLOC_BUILD=1")
set_property(TARGET tbbmalloc APPEND PROPERTY COMPILE_DEFINITIONS "__TBB_DYNAMIC_LOAD_ENABLED=0")
set_property(TARGET tbbmalloc APPEND PROPERTY COMPILE_DEFINITIONS "__TBB_SOURCE_DIRECTLY_INCLUDED=1")
set_property(TARGET tbbmalloc APPEND_STRING PROPERTY COMPILE_FLAGS ${DISABLE_RTTI})
if (MSVC)
  target_compile_definitions(tbbmalloc PUBLIC __TBB_NO_IMPLICIT_LINKAGE=1 __TBBMALLOC_NO_IMPLICIT_LINKAGE=1)
endif()

add_library(tbbmalloc_proxy STATIC ${tbbmalloc_proxy_src})
target_link_libraries(tbbmalloc_proxy PRIVATE tbb_interface)
set_property(TARGET tbbmalloc_proxy APPEND PROPERTY COMPILE_DEFINITIONS "__TBBMALLOC_BUILD=1")
set_property(TARGET tbbmalloc_proxy APPEND PROPERTY COMPILE_DEFINITIONS "__TBB_DYNAMIC_LOAD_ENABLED=0")
set_property(TARGET tbbmalloc_proxy APPEND PROPERTY COMPILE_DEFINITIONS "__TBB_SOURCE_DIRECTLY_INCLUDED=1")
set_property(TARGET tbbmalloc_proxy APPEND_STRING PROPERTY COMPILE_FLAGS ${DISABLE_RTTI})

# version_string.ver
if (UNIX AND NOT TBB_NO_DATE)
  execute_process (COMMAND date "+%a, %d %b %Y %H:%M:%S %z"
                   OUTPUT_VARIABLE _configure_date
                   OUTPUT_STRIP_TRAILING_WHITESPACE)
elseif (WIN32 AND NOT TBB_NO_DATE)
  execute_process (COMMAND cmd " /C date /T"
                   OUTPUT_VARIABLE _configure_date
                   OUTPUT_STRIP_TRAILING_WHITESPACE)
else ()
  set (_configure_date "Unknown")
endif()
set (TBB_CONFIG_DATE "${_configure_date}" CACHE STRING "First time that TBB was configured")
set (_configure_date "${TBB_CONFIG_DATE}")
include_directories (${CMAKE_CURRENT_BINARY_DIR})
configure_file (build/version_string.ver.in ${CMAKE_CURRENT_BINARY_DIR}/version_string.ver @ONLY)

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(tbb PROPERTIES FOLDER external)
  SET_TARGET_PROPERTIES(tbbmalloc PROPERTIES FOLDER external)
  SET_TARGET_PROPERTIES(tbbmalloc_proxy PROPERTIES FOLDER external)
  SET_TARGET_PROPERTIES(tbb_def_files PROPERTIES FOLDER external)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
