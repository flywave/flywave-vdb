# Copyright Contributors to the OpenImageIO project.
# SPDX-License-Identifier: Apache-2.0
# https://github.com/AcademySoftwareFoundation/OpenImageIO

cmake_minimum_required (VERSION 3.18.2...4.0)

set (OpenImageIO_VERSION "3.0.13.0")
set (OpenImageIO_VERSION_OVERRIDE "" CACHE STRING
     "Version override (use with caution)!")
mark_as_advanced (OpenImageIO_VERSION_OVERRIDE)
if (OpenImageIO_VERSION_OVERRIDE)
    set (OpenImageIO_VERSION ${OpenImageIO_VERSION_OVERRIDE})
endif ()

project (OpenImageIO VERSION ${OpenImageIO_VERSION}
         HOMEPAGE_URL "https://openimageio.org"
         LANGUAGES CXX C)

set (PROJ_NAME OIIO)    # short name, caps
string (TOLOWER ${PROJ_NAME} PROJ_NAME_LOWER)  # short name lower case
string (TOUPPER ${PROJ_NAME} PROJ_NAME_UPPER)  # short name upper case
set (PROJECT_VERSION_RELEASE_TYPE "" CACHE STRING
    "Build type, for example: dev, beta2, RC1 (empty string for normal release)")
set (${PROJECT_NAME}_VERSION_RELEASE_TYPE ${PROJECT_VERSION_RELEASE_TYPE})
set (PROJECT_AUTHORS "Contributors to the OpenImageIO project")
option (${PROJECT_NAME}_SUPPORTED_RELEASE
        "Set ON for supported release branches, OFF for 'main'" ON)
if (${PROJECT_NAME}_SUPPORTED_RELEASE)
    set (${PROJECT_NAME}_DEV_RELEASE OFF)
else ()
    set (${PROJECT_NAME}_DEV_RELEASE ON)
endif ()

# Set PROJECT_IS_TOP_LEVEL to ON if if this is the top level project (not
# if this is included as a subproject of something else). Note that this is
# handled automatically for CMake >= 3.21.
if (CMAKE_VERSION VERSION_LESS 3.21)
    if ("${CMAKE_PROJECT_NAME}" STREQUAL "${PROJECT_NAME}")
        set (PROJECT_IS_TOP_LEVEL ON)
    endif ()
endif ()

# Set up module path for our own cmake modules and add some esential ones
list (APPEND CMAKE_MODULE_PATH
      "${PROJECT_SOURCE_DIR}/src/cmake/modules"
      "${PROJECT_SOURCE_DIR}/src/cmake")

# Utilities
include (colors)
include (set_utils)
include (check_is_enabled)
include (fancy_add_executable)

# If the user wants to use Conan to build dependencies, they will have done
# this prior to the cmake config:
#   cd <build area>
#   conan install <source area>
# and that will leave a conanbuildinfo.cmake in the build area for us.
if (EXISTS ${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
    include (${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
    message (STATUS "Using Conan for dependencies")
    conan_basic_setup()
endif()

if (NOT CMAKE_BUILD_TYPE)
    set (CMAKE_BUILD_TYPE "Release")
endif ()

# If the user hasn't configured cmake with an explicit
# -DCMAKE_INSTALL_PREFIX=..., then set it to safely install into ./dist, to
# help prevent the user from accidentally writing over /usr/local or whatever.
if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT
      AND PROJECT_IS_TOP_LEVEL)
    set (CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/dist" CACHE PATH
         "Installation location" FORCE)
endif()

message (STATUS "Configuring ${PROJECT_NAME} ${PROJECT_VERSION}")
message (STATUS "CMake ${CMAKE_VERSION}")
message (STATUS "CMake system           = ${CMAKE_SYSTEM}")
message (STATUS "CMake system name      = ${CMAKE_SYSTEM_NAME}")
message (STATUS "Project source dir     = ${PROJECT_SOURCE_DIR}")
message (STATUS "Project build dir      = ${CMAKE_BINARY_DIR}")
message (STATUS "Project install prefix = ${CMAKE_INSTALL_PREFIX}")
message (STATUS "Configuration types    = ${CMAKE_CONFIGURATION_TYPES}")
message (STATUS "Build type             = ${CMAKE_BUILD_TYPE}")
message (STATUS "Supported release      = ${${PROJECT_NAME}_SUPPORTED_RELEASE}")

# Make the build area layout look a bit more like the final dist layout
if (PROJECT_IS_TOP_LEVEL)
    set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()

if ("${PROJECT_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
    message (FATAL_ERROR "Not allowed to run in-source build!")
endif ()

option (CMAKE_USE_FOLDERS "Use the FOLDER target property to organize targets into folders." ON)
mark_as_advanced (CMAKE_USE_FOLDERS)
if (CMAKE_USE_FOLDERS)
    set_property (GLOBAL PROPERTY USE_FOLDERS ON)
endif ()


option (VERBOSE "Print lots of messages while compiling" OFF)
if (VERBOSE)
    set (CMAKE_MESSAGE_LOG_LEVEL "VERBOSE" CACHE STRING "CMake log level to display")
else ()
    set (CMAKE_MESSAGE_LOG_LEVEL "STATUS" CACHE STRING "CMake log level to display")
endif ()
option (${PROJ_NAME}_BUILD_TOOLS "Build the command-line tools" OFF)
option (${PROJ_NAME}_BUILD_TESTS "Build the unit tests" OFF)
set (OIIO_LIBNAME_SUFFIX "" CACHE STRING
     "Optional name appended to ${PROJECT_NAME} libraries that are built")
option (BUILD_OIIOUTIL_ONLY "If ON, will build *only* libOpenImageIO_Util" OFF)
option (BUILD_DOCS "If ON, build documentation and man pages." OFF)
option (INSTALL_DOCS "If ON, install documentation and man pages." OFF)
option (INSTALL_FONTS "If ON, install default fonts" OFF)
option (EMBEDPLUGINS "Embed format plugins in libOpenImageIO" ON)
set (PLUGIN_SEARCH_PATH "" CACHE STRING "Default plugin search path")
file (TO_NATIVE_PATH "${PLUGIN_SEARCH_PATH}" PLUGIN_SEARCH_PATH_NATIVE)
set (CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "Library naming postfix for Debug builds")
set(USE_JPEGTURBO FALSE)
set(USE_OPENCV FALSE)
set(USE_OPENSSL FALSE)
set(USE_FREETYPE FALSE)
set(USE_GIF FALSE)
set(USE_PTEX FALSE)
set(USE_LIBRAW FALSE)
set(USE_FFMPEG FALSE)
if (CMAKE_UNITY_BUILD_BATCH_SIZE)
    set (UNITY_SMALL_BATCH_SIZE "${CMAKE_UNITY_BUILD_BATCH_SIZE}" CACHE STRING "Unity batch mode size for expensive files")
else ()
    set (UNITY_SMALL_BATCH_SIZE 8 CACHE STRING "Unity batch mode size for expensive files")
endif ()
message(STATUS "CMAKE_UNITY_BUILD_MODE = ${CMAKE_UNITY_BUILD_MODE}")
message(STATUS "CMAKE_UNITY_BUILD_BATCH_SIZE = ${CMAKE_UNITY_BUILD_BATCH_SIZE}")

option (OIIO_THREAD_ALLOW_DCLP "OIIO threads may use DCLP for speed" ON)
if (NOT OIIO_THREAD_ALLOW_DCLP)
    add_compile_definitions (OIIO_THREAD_ALLOW_DCLP=0)
endif ()

set (TEX_BATCH_SIZE "" CACHE STRING "Force TextureSystem SIMD batch size (e.g. 16)")
if (TEX_BATCH_SIZE)
    add_compile_definitions (OIIO_TEXTURE_SIMD_BATCH_WIDTH=${TEX_BATCH_SIZE})
endif ()

# Set the default namespace
set (${PROJ_NAME}_NAMESPACE ${PROJECT_NAME} CACHE STRING
     "Customized outer namespace base name (version will be added)")
option (${PROJ_NAME}_NAMESPACE_INCLUDE_PATCH
        "Should the inner namespace include the patch number" ${${PROJECT_NAME}_DEV_RELEASE})
set (PROJ_NAMESPACE "${${PROJ_NAME}_NAMESPACE}")
set (PROJ_NAMESPACE_V "${PROJ_NAMESPACE}_v${PROJECT_VERSION_MAJOR}_${PROJECT_VERSION_MINOR}")
if (${PROJ_NAME}_NAMESPACE_INCLUDE_PATCH)
    set (PROJ_NAMESPACE_V "${PROJ_NAMESPACE_V}_${PROJECT_VERSION_PATCH}")
endif ()
message(STATUS "Setting Namespace to: ${PROJ_NAMESPACE_V}")


# Define OIIO_INTERNAL symbol only when building OIIO itself, will not be
# defined for downstream projects using OIIO.
add_compile_definitions (OIIO_INTERNAL=1)

# If CMake variable OIIO_SITE is set, set compile symbol OIIO_SITE_sitename
if (OIIO_SITE)
    add_compile_definitions (OIIO_SITE_${OIIO_SITE})
endif ()


# Utilities for build instructions of the format-reading plugins
include (add_oiio_plugin)

# All the C++ and compiler related options and adjustments
include (compiler)

option (IGNORE_HOMEBREWED_DEPS "If ON, will ignore homebrew-installed dependencies" OFF)
if (IGNORE_HOMEBREWED_DEPS)
    # Define the list of prefixes to ignore
    set (HOMEBREW_PREFIXES
         /opt/homebrew
         /opt/homebrew/Cellar
         /usr/local
         /usr/local/Cellar
         /usr/X11
         /usr/X11R6
         /opt/X11
    )
    message (STATUS "Ignoring Homebrew dependencies and adjusted environment and CMake variables accordingly.")
    foreach (_cmake_var
             CMAKE_SYSTEM_INCLUDE_PATH
             CMAKE_SYSTEM_LIBRARY_PATH
             CMAKE_PREFIX_PATH)
        remove_prefixes_from_variable (CMAKE ${_cmake_var} "${HOMEBREW_PREFIXES}")
    endforeach ()

    # Adjust CMAKE_IGNORE_PATH
    foreach (_prefix IN LISTS HOMEBREW_PREFIXES)
        list (APPEND CMAKE_IGNORE_PATH
              "${_prefix}"
              "${_prefix}/lib"
              "${_prefix}/bin"
              "${_prefix}/include"
             )
    endforeach ()

    message (STATUS "CMAKE_IGNORE_PATH: ${CMAKE_IGNORE_PATH}")
endif ()


include (cuda_macros)

IF (WIN32)
  INCLUDE_DIRECTORIES(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/../libgdal/arch/win/)
ELSE()
  INCLUDE_DIRECTORIES(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/../libgdal/arch/unix/)
ENDIF()
INCLUDE_DIRECTORIES(
    "${CMAKE_CURRENT_SOURCE_DIR}/../libexpat/expat/lib"
    "${CMAKE_CURRENT_SOURCE_DIR}/../freetype2/include/"
    "${CMAKE_CURRENT_SOURCE_DIR}/../freetype2/src/include/"
    "${CMAKE_CURRENT_SOURCE_DIR}/../fontconfig/src/"
    "${CMAKE_CURRENT_SOURCE_DIR}/../fontconfig/include/"
    "${CMAKE_CURRENT_SOURCE_DIR}/../libjpeg/"
    "${CMAKE_CURRENT_BINARY_DIR}/../libjpeg/"
    "${CMAKE_CURRENT_SOURCE_DIR}/../libpng/"
    "${CMAKE_CURRENT_SOURCE_DIR}/../libiconv/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../webp/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../libgdal/gdal/frmts/gtiff/libtiff"
    "${CMAKE_CURRENT_SOURCE_DIR}/../libgdal/gdal/port"
    "${CMAKE_CURRENT_SOURCE_DIR}/../libjasper"
    "${CMAKE_CURRENT_SOURCE_DIR}/../openexr/src/lib"
    "${CMAKE_CURRENT_BINARY_DIR}/../openexr"
    "${CMAKE_CURRENT_SOURCE_DIR}/../openexr/src/lib/Imf"
    "${CMAKE_CURRENT_SOURCE_DIR}/../openexr/src/lib/OpenEXR"
    "${CMAKE_CURRENT_SOURCE_DIR}/../openexr/src/lib/OpenEXRUtil"
    "${CMAKE_CURRENT_SOURCE_DIR}/../openexr/src/lib/OpenEXRCore"
    "${CMAKE_CURRENT_SOURCE_DIR}/../openexr/src/lib/IlmThread"
    "${CMAKE_CURRENT_SOURCE_DIR}/../openexr/src/lib/Iex"
    "${CMAKE_CURRENT_SOURCE_DIR}/../imath/src/"
    "${CMAKE_CURRENT_BINARY_DIR}/../imath"
    "${CMAKE_CURRENT_SOURCE_DIR}/../imath/src/Imath"
    "${CMAKE_CURRENT_SOURCE_DIR}/../libboost/boost_1_67_0"
    "${CMAKE_CURRENT_SOURCE_DIR}/../fmt/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../robin-map/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../zlib"
    "${CMAKE_CURRENT_BINARY_DIR}/../zlib"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ocio/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ocio/include/OpenColorIO"
    "${CMAKE_CURRENT_BINARY_DIR}/../ocio/include"
    "${CMAKE_CURRENT_BINARY_DIR}/../ocio/include/OpenColorIO"
)


include_directories (
    BEFORE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/include"
    "${CMAKE_CURRENT_BINARY_DIR}/include"
    "${CMAKE_CURRENT_BINARY_DIR}/include/OpenImageIO"
  )

# Tell CMake to process the sub-directories
add_subdirectory (src/libutil)

# Add IO plugin directories -- if we are embedding plugins, we need to visit
# these directories BEFORE the OpenImageIO target is established (in
# src/libOpenImageIO). For each plugin, we append to the lists of source
# files, format libs, include directories, and definitions, all of which
# will be used by src/libOpenImageIO.
set (libOpenImageIO_srcs "")
set (format_plugin_libs "")
set (format_plugin_include_dirs "")
set (format_plugin_definitions "")
file (GLOB all_format_plugin_dirs src/*.imageio)
if (EMBEDPLUGINS AND NOT BUILD_OIIOUTIL_ONLY)
    foreach (plugin_dir ${all_format_plugin_dirs})
        add_subdirectory (${plugin_dir})
    endforeach ()
endif ()

if (NOT BUILD_OIIOUTIL_ONLY)
    add_subdirectory (src/libOpenImageIO)
endif ()

# Add IO plugin directories -- if we are not embedding plugins, we need to
# do it AFTER the OpenImageIO target is established (in src/libOpenImageIO),
# since each plugin needs libOpenImageIO to be a dependency.
if (NOT EMBEDPLUGINS AND NOT BUILD_OIIOUTIL_ONLY)
    foreach (plugin_dir ${all_format_plugin_dirs})
        add_subdirectory (${plugin_dir})
    endforeach ()
endif ()

add_subdirectory (src/include)
add_subdirectory (src/fonts)
