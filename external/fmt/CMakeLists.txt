cmake_minimum_required(VERSION 3.8...3.28)

# Fallback for using newer policies on CMake <3.12.
if (${CMAKE_VERSION} VERSION_LESS 3.12)
  cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif ()

# Determine if fmt is built as a subproject (using add_subdirectory)
# or if it is the master project.
if (NOT DEFINED FMT_MASTER_PROJECT)
  set(FMT_MASTER_PROJECT OFF)
  # NOTE: source vs current_source detection is unreliable
  # this heuristic is more generally applicable esp w.r.t FetchContent
  if (NOT DEFINED PROJECT_NAME)
    set(FMT_MASTER_PROJECT ON)
    message(STATUS "CMake version: ${CMAKE_VERSION}")
  endif ()
endif ()

# Joins arguments and places the results in ${result_var}.
function(join result_var)
  set(result "")
  foreach (arg ${ARGN})
    set(result "${result}${arg}")
  endforeach ()
  set(${result_var} "${result}" PARENT_SCOPE)
endfunction()

# DEPRECATED! Should be merged into add_module_library.
function(enable_module target)
  if (MSVC)
    if(NOT CMAKE_GENERATOR STREQUAL "Ninja")
      set(BMI_DIR "${CMAKE_CURRENT_BINARY_DIR}")
      file(TO_NATIVE_PATH "${BMI_DIR}/${target}.ifc" BMI)
      target_compile_options(${target}
        PRIVATE /interface /ifcOutput ${BMI}
        INTERFACE /reference fmt=${BMI})
      set_target_properties(${target} PROPERTIES ADDITIONAL_CLEAN_FILES ${BMI})
      set_source_files_properties(${BMI} PROPERTIES GENERATED ON)
    endif()
  endif ()
endfunction()

set(FMT_USE_CMAKE_MODULES FALSE)
if (CMAKE_VERSION VERSION_GREATER_EQUAL 3.28 AND
    CMAKE_GENERATOR STREQUAL "Ninja")
  set(FMT_USE_CMAKE_MODULES TRUE)
endif ()

# Adds a library compiled with C++20 module support.
# `enabled` is a CMake variables that specifies if modules are enabled.
# If modules are disabled `add_module_library` falls back to creating a
# non-modular library.
#
# Usage:
#   add_module_library(<name> [sources...] FALLBACK [sources...] [IF enabled])
function(add_module_library name)
  cmake_parse_arguments(AML "" "IF" "FALLBACK" ${ARGN})
  set(sources ${AML_UNPARSED_ARGUMENTS})

  add_library(${name})
  set_target_properties(${name} PROPERTIES LINKER_LANGUAGE CXX)

  if (NOT ${${AML_IF}})
    # Create a non-modular library.
    target_sources(${name} PRIVATE ${AML_FALLBACK})
    set_target_properties(${name} PROPERTIES CXX_SCAN_FOR_MODULES OFF)
    return()
  endif ()

  # Modules require C++20.
  target_compile_features(${name} PUBLIC cxx_std_20)
  if (CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(${name} PUBLIC -fmodules-ts)
  endif ()

  if (FMT_USE_CMAKE_MODULES)
    target_sources(${name} PUBLIC FILE_SET fmt TYPE CXX_MODULES
                   FILES ${sources})
  else()
    # `std` is affected by CMake options and may be higher than C++20.
    get_target_property(std ${name} CXX_STANDARD)

    if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      set(pcms)
      foreach (src ${sources})
        get_filename_component(pcm ${src} NAME_WE)
        set(pcm ${pcm}.pcm)

        # Propagate -fmodule-file=*.pcm to targets that link with this library.
        target_compile_options(
          ${name} PUBLIC -fmodule-file=${CMAKE_CURRENT_BINARY_DIR}/${pcm})

        # Use an absolute path to prevent target_link_libraries prepending -l
        # to it.
        set(pcms ${pcms} ${CMAKE_CURRENT_BINARY_DIR}/${pcm})
        add_custom_command(
          OUTPUT ${pcm}
          COMMAND ${CMAKE_CXX_COMPILER}
                  -std=c++${std} -x c++-module --precompile -c
                  -o ${pcm} ${CMAKE_CURRENT_SOURCE_DIR}/${src}
                  "-I$<JOIN:$<TARGET_PROPERTY:${name},INCLUDE_DIRECTORIES>,;-I>"
          # Required by the -I generator expression above.
          COMMAND_EXPAND_LISTS
          DEPENDS ${src})
      endforeach ()

      # Add .pcm files as sources to make sure they are built before the library.
      set(sources)
      foreach (pcm ${pcms})
        get_filename_component(pcm_we ${pcm} NAME_WE)
        set(obj ${pcm_we}.o)
        # Use an absolute path to prevent target_link_libraries prepending -l.
        set(sources ${sources} ${pcm} ${CMAKE_CURRENT_BINARY_DIR}/${obj})
        add_custom_command(
          OUTPUT ${obj}
          COMMAND ${CMAKE_CXX_COMPILER} $<TARGET_PROPERTY:${name},COMPILE_OPTIONS>
                  -c -o ${obj} ${pcm}
          DEPENDS ${pcm})
      endforeach ()
    endif ()
    target_sources(${name} PRIVATE ${sources})
  endif()
endfunction()

include(CMakeParseArguments)

# Sets a cache variable with a docstring joined from multiple arguments:
#   set(<variable> <value>... CACHE <type> <docstring>...)
# This allows splitting a long docstring for readability.
function(set_verbose)
  # cmake_parse_arguments is broken in CMake 3.4 (cannot parse CACHE) so use
  # list instead.
  list(GET ARGN 0 var)
  list(REMOVE_AT ARGN 0)
  list(GET ARGN 0 val)
  list(REMOVE_AT ARGN 0)
  list(REMOVE_AT ARGN 0)
  list(GET ARGN 0 type)
  list(REMOVE_AT ARGN 0)
  join(doc ${ARGN})
  set(${var} ${val} CACHE ${type} ${doc})
endfunction()

# Set the default CMAKE_BUILD_TYPE to Release.
# This should be done before the project command since the latter can set
# CMAKE_BUILD_TYPE itself (it does so for nmake).
if (FMT_MASTER_PROJECT AND NOT CMAKE_BUILD_TYPE)
  set_verbose(CMAKE_BUILD_TYPE Release CACHE STRING
              "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or "
              "CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
endif ()

project(FMT CXX)
include(GNUInstallDirs)
set_verbose(FMT_INC_DIR ${CMAKE_INSTALL_INCLUDEDIR} CACHE STRING
            "Installation directory for include files, a relative path that "
            "will be joined with ${CMAKE_INSTALL_PREFIX} or an absolute path.")

option(FMT_PEDANTIC "Enable extra warnings and expensive tests." OFF)
option(FMT_WERROR "Halt the compilation with an error on compiler warnings."
       OFF)

# Options that control generation of various targets.
option(FMT_DOC "Generate the doc target." ${FMT_MASTER_PROJECT})
option(FMT_INSTALL "Generate the install target." ${FMT_MASTER_PROJECT})
option(FMT_TEST "Generate the test target." ${FMT_MASTER_PROJECT})
option(FMT_FUZZ "Generate the fuzz target." OFF)
option(FMT_CUDA_TEST "Generate the cuda-test target." OFF)
option(FMT_OS "Include OS-specific APIs." ON)
option(FMT_MODULE "Build a module instead of a traditional library." OFF)
option(FMT_SYSTEM_HEADERS "Expose headers with marking them as system." OFF)
option(FMT_UNICODE "Enable Unicode support." ON)

if (FMT_TEST AND FMT_MODULE)
  # The tests require {fmt} to be compiled as traditional library
  message(STATUS "Testing is incompatible with build mode 'module'.")
endif ()
set(FMT_SYSTEM_HEADERS_ATTRIBUTE "")
if (FMT_SYSTEM_HEADERS)
  set(FMT_SYSTEM_HEADERS_ATTRIBUTE SYSTEM)
endif ()
if (CMAKE_SYSTEM_NAME STREQUAL "MSDOS")
  set(FMT_TEST OFF)
  message(STATUS "MSDOS is incompatible with gtest")
endif ()

# Get version from base.h
file(READ include/fmt/base.h base_h)
if (NOT base_h MATCHES "FMT_VERSION ([0-9]+)([0-9][0-9])([0-9][0-9])")
  message(FATAL_ERROR "Cannot get FMT_VERSION from base.h.")
endif ()
# Use math to skip leading zeros if any.
math(EXPR CPACK_PACKAGE_VERSION_MAJOR ${CMAKE_MATCH_1})
math(EXPR CPACK_PACKAGE_VERSION_MINOR ${CMAKE_MATCH_2})
math(EXPR CPACK_PACKAGE_VERSION_PATCH ${CMAKE_MATCH_3})
join(FMT_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.
                 ${CPACK_PACKAGE_VERSION_PATCH})
message(STATUS "{fmt} version: ${FMT_VERSION}")

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if (NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
endif ()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/support/cmake")

include(CheckCXXCompilerFlag)

if (FMT_MASTER_PROJECT AND NOT DEFINED CMAKE_CXX_VISIBILITY_PRESET)
  set_verbose(CMAKE_CXX_VISIBILITY_PRESET hidden CACHE STRING
              "Preset for the export of private symbols")
  set_property(CACHE CMAKE_CXX_VISIBILITY_PRESET PROPERTY STRINGS
               hidden default)
endif ()

if (FMT_MASTER_PROJECT AND NOT DEFINED CMAKE_VISIBILITY_INLINES_HIDDEN)
  set_verbose(CMAKE_VISIBILITY_INLINES_HIDDEN ON CACHE BOOL
              "Whether to add a compile flag to hide symbols of inline functions")
endif ()

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  set(PEDANTIC_COMPILE_FLAGS -pedantic-errors -Wall -Wextra -pedantic
      -Wold-style-cast -Wundef
      -Wredundant-decls -Wwrite-strings -Wpointer-arith
      -Wcast-qual -Wformat=2 -Wmissing-include-dirs
      -Wcast-align
      -Wctor-dtor-privacy -Wdisabled-optimization
      -Winvalid-pch -Woverloaded-virtual
      -Wconversion -Wundef
      -Wno-ctor-dtor-privacy -Wno-format-nonliteral)
  if (NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.6)
      set(PEDANTIC_COMPILE_FLAGS ${PEDANTIC_COMPILE_FLAGS}
         -Wno-dangling-else -Wno-unused-local-typedefs)
  endif ()
  if (NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.0)
      set(PEDANTIC_COMPILE_FLAGS ${PEDANTIC_COMPILE_FLAGS} -Wdouble-promotion
          -Wtrampolines -Wzero-as-null-pointer-constant -Wuseless-cast
          -Wvector-operation-performance -Wsized-deallocation -Wshadow)
  endif ()
  if (NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 6.0)
      set(PEDANTIC_COMPILE_FLAGS ${PEDANTIC_COMPILE_FLAGS} -Wshift-overflow=2
          -Wduplicated-cond)
      # Workaround for GCC regression
      # [12/13/14/15 regression] New (since gcc 12) false positive null-dereference in vector.resize
      # https://gcc.gnu.org/bugzilla/show_bug.cgi?id=108860
      if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 12.0)
        set(PEDANTIC_COMPILE_FLAGS ${PEDANTIC_COMPILE_FLAGS} -Wnull-dereference)
      endif ()
  endif ()
  set(WERROR_FLAG -Werror)
endif ()

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(PEDANTIC_COMPILE_FLAGS -Wall -Wextra -pedantic -Wconversion -Wundef
      -Wdeprecated -Wweak-vtables -Wshadow
      -Wno-gnu-zero-variadic-macro-arguments)
  check_cxx_compiler_flag(-Wzero-as-null-pointer-constant HAS_NULLPTR_WARNING)
  if (HAS_NULLPTR_WARNING)
    set(PEDANTIC_COMPILE_FLAGS ${PEDANTIC_COMPILE_FLAGS}
        -Wzero-as-null-pointer-constant)
  endif ()
  set(WERROR_FLAG -Werror)
endif ()

if (MSVC)
  set(PEDANTIC_COMPILE_FLAGS /W3)
  set(WERROR_FLAG /WX)
endif ()

if (FMT_MASTER_PROJECT AND CMAKE_GENERATOR MATCHES "Visual Studio")
  # If Microsoft SDK is installed create script run-msbuild.bat that
  # calls SetEnv.cmd to set up build environment and runs msbuild.
  # It is useful when building Visual Studio projects with the SDK
  # toolchain rather than Visual Studio.
  include(FindSetEnv)
  if (WINSDK_SETENV)
    set(MSBUILD_SETUP "call \"${WINSDK_SETENV}\"")
  endif ()
  # Set FrameworkPathOverride to get rid of MSB3644 warnings.
  join(netfxpath
       "C:\\Program Files\\Reference Assemblies\\Microsoft\\Framework\\"
       ".NETFramework\\v4.0")
  file(WRITE run-msbuild.bat "
    ${MSBUILD_SETUP}
    ${CMAKE_MAKE_PROGRAM} -p:FrameworkPathOverride=\"${netfxpath}\" %*")
endif ()

function(add_headers VAR)
  set(headers ${${VAR}})
  foreach (header ${ARGN})
    set(headers ${headers} include/fmt/${header})
  endforeach()
  set(${VAR} ${headers} PARENT_SCOPE)
endfunction()

# Define the fmt library, its includes and the needed defines.
set(FMT_HEADERS)
add_headers(FMT_HEADERS args.h base.h chrono.h color.h compile.h core.h format.h
                        format-inl.h os.h ostream.h printf.h ranges.h std.h
                        xchar.h)
set(FMT_SOURCES src/format.cc)

add_library(fmt STATIC ${FMT_SOURCES} ${FMT_HEADERS})

if (FMT_WERROR)
  target_compile_options(fmt PRIVATE ${WERROR_FLAG})
endif ()
if (FMT_PEDANTIC)
  target_compile_options(fmt PRIVATE ${PEDANTIC_COMPILE_FLAGS})
endif ()

target_include_directories(fmt PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

set_target_properties(fmt PROPERTIES
  VERSION ${FMT_VERSION} SOVERSION ${CPACK_PACKAGE_VERSION_MAJOR})

SOURCE_GROUP("src" FILES ${FMT_SOURCES})
SOURCE_GROUP("include" FILES ${FMT_HEADERS})

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(fmt PROPERTIES FOLDER external)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)

SET_TARGET_PROPERTIES(fmt PROPERTIES
ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}
ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR})
SET_TARGET_PROPERTIES(fmt PROPERTIES 
LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}
LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR})
SET_TARGET_PROPERTIES(fmt PROPERTIES
RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}
RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR})