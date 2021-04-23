cmake_minimum_required(VERSION 3.9.0)

# Default policy is from 3.9.0
cmake_policy(VERSION 3.9.0)

# Find CMake modules in cmake/
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Read version information from configure.ac.
message(STATUS "Building tiff version ${LIBTIFF_VERSION_FULL}")
message(STATUS "libtiff library version ${SO_VERSION}")
string(TIMESTAMP BUILD_DATE "%Y%m%d")
message(STATUS "libtiff build date: ${BUILD_DATE}")

# Disable deprecated features to ensure clean build
add_definitions(-DTIFF_DISABLE_DEPRECATED)

INCLUDE_DIRECTORIES(
    "${CMAKE_CURRENT_SOURCE_DIR}/../libjpeg/"
    "${CMAKE_CURRENT_BINARY_DIR}/../libjpeg/"
    "${CMAKE_CURRENT_SOURCE_DIR}/../zlib"
    "${CMAKE_CURRENT_SOURCE_DIR}/../lerc/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../webp/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../zstd/lib"
    "${CMAKE_CURRENT_SOURCE_DIR}/../lzma/api/"
)

# Project definition
set(CMAKE_C_STANDARD 99)
project(tiff
        VERSION "${LIBTIFF_VERSION}"
        LANGUAGES C CXX)

IF(UNIX)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-implicit-function-declaration ")
ENDIF()

# Autotools compatibility
include(AutotoolsCompat)

# Compiler checks (warning flags, etc.)
include(CompilerChecks)

# Linker checks (version script, etc.)
include(LinkerChecks)

# Processor checks (endianness, fill order, floating point, etc.)
include(ProcessorChecks)

# Checks for headers
include(IncludeChecks)

# Checks for functions and other symbols
include(SymbolChecks)

# Documentation install directory (default to cmake project docdir)
set(LIBTIFF_DOCDIR "${CMAKE_INSTALL_FULL_DOCDIR}")

# Checks for type presence and size
include(TypeSizeChecks)

# Check for POSIX Large File Support (LFS)
include(LargeFileSupport)

# Options for internal codec support
include(InternalCodecs)

set(ZLIB_SUPPORT TRUE)
set(PIXARLOG_SUPPORT TRUE)
set(LIBDEFLATE_SUPPORT FALSE)
set(JPEG_SUPPORT TRUE)
set(LERC_SUPPORT TRUE)
set(LZMA_SUPPORT TRUE)
set(ZSTD_SUPPORT TRUE)
set(ZSTD_USABLE TRUE)
set(WEBP_SUPPORT TRUE)
set(HAVE_OPENGL FALSE)
set(CXX_SUPPORT TRUE)
set(win32_io_default OFF)


# Orthogonal features
include(LibraryFeatures)

# math.h/libm portability
find_package(CMath REQUIRED)

# Process subdirectories
add_subdirectory(port)
add_subdirectory(src)
