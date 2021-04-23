cmake_minimum_required(VERSION 2.8.0)
cmake_policy(VERSION 2.8)

cmake_policy(SET CMP0003 NEW)
if (POLICY CMP0042)
  cmake_policy(SET CMP0042 NEW)
endif()

project(Ceres C CXX)

list(APPEND CMAKE_MODULE_PATH "${Ceres_SOURCE_DIR}/cmake")
include(UpdateCacheVariable)

# Set up the git hook to make Gerrit Change-Id: lines in commit messages.
include(AddGerritCommitHook)
add_gerrit_commit_hook(${Ceres_SOURCE_DIR} ${Ceres_BINARY_DIR})

if (CMAKE_SYSTEM_NAME MATCHES "Darwin")
  find_program(HOMEBREW_EXECUTABLE brew)
  mark_as_advanced(FORCE HOMEBREW_EXECUTABLE)
  if (HOMEBREW_EXECUTABLE)
    execute_process(COMMAND ${HOMEBREW_EXECUTABLE} --prefix
      OUTPUT_VARIABLE HOMEBREW_INSTALL_PREFIX
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    message(STATUS "Detected Homebrew with install prefix: "
      "${HOMEBREW_INSTALL_PREFIX}, adding to CMake search paths.")
    list(APPEND CMAKE_PREFIX_PATH "${HOMEBREW_INSTALL_PREFIX}")
  endif()
endif()

include(ReadCeresVersionFromSource)
read_ceres_version_from_source(${Ceres_SOURCE_DIR})

include(CMakeDependentOption)
option(MINIGLOG "Use a stripped down version of glog." ON)
option(GFLAGS "Enable Google Flags." ON)
option(SUITESPARSE "Enable SuiteSparse." ON)
option(CXSPARSE "Enable CXSparse." OFF)
option(LAPACK "Enable use of LAPACK directly within Ceres." OFF)
option(LAPACKE "Enable use of LAPACKE directly within Ceres." OFF)
if (APPLE)
  option(ACCELERATESPARSE
    "Enable use of sparse solvers in Apple's Accelerate framework." ON)
endif()

option(SCHUR_SPECIALIZATIONS "Enable fixed-size schur specializations." ON)
option(CUSTOM_BLAS
       "Use handcoded BLAS routines (usually faster) instead of Eigen."
     OFF)

option(CXX11 "Enable use of C++11 headers if available (requires client code use C++11)." ON)

cmake_dependent_option(
  CXX11_THREADS "Enable threaded solving in Ceres with C++11 primitives" OFF
  "CXX11;NOT OPENMP;NOT TBB" OFF)

option(EIGENSPARSE "Enable Eigen as a sparse linear algebra library." OFF)

if (MSVC)
  update_cache_variable(CXX11 ON)

  option(MSVC_USE_STATIC_CRT
    "MS Visual Studio: Use static C-Run Time Library in place of shared." OFF)
endif (MSVC)

unset(CERES_COMPILE_OPTIONS)

add_definitions(-DCERES_NO_CXSPARSE)
add_definitions(-DEIGEN_MPL2_ONLY)

  set(GLOG_INCLUDE_DIRS internal/ceres/miniglog)
  set(MINIGLOG_MAX_LOG_LEVEL 2 CACHE STRING "The maximum message severity level to be logged")
  add_definitions("-DMAX_LOG_LEVEL=${MINIGLOG_MAX_LOG_LEVEL}")

  mark_as_advanced(FORCE GLOG_INCLUDE_DIR
                         GLOG_LIBRARY)

if (NOT SCHUR_SPECIALIZATIONS)
  list(APPEND CERES_COMPILE_OPTIONS CERES_RESTRICT_SCHUR_SPECIALIZATION)
endif (NOT SCHUR_SPECIALIZATIONS)

set(CMAKE_REQUIRED_FLAGS ${CMAKE_CXX_FLAGS})
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_HAS_CXX11_FLAG)
if (CXX11 AND COMPILER_HAS_CXX11_FLAG)
  set(CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS} -std=c++11")
endif (CXX11 AND COMPILER_HAS_CXX11_FLAG)

include(FindUnorderedMap)
find_unordered_map()
if (UNORDERED_MAP_FOUND)
  if (HAVE_UNORDERED_MAP_IN_STD_NAMESPACE)
    list(APPEND CERES_COMPILE_OPTIONS CERES_STD_UNORDERED_MAP)
  endif(HAVE_UNORDERED_MAP_IN_STD_NAMESPACE)
  if (HAVE_UNORDERED_MAP_IN_TR1_NAMESPACE)
    list(APPEND CERES_COMPILE_OPTIONS CERES_STD_UNORDERED_MAP_IN_TR1_NAMESPACE)
  endif(HAVE_UNORDERED_MAP_IN_TR1_NAMESPACE)
  if (HAVE_TR1_UNORDERED_MAP_IN_TR1_NAMESPACE)
    list(APPEND CERES_COMPILE_OPTIONS CERES_TR1_UNORDERED_MAP)
  endif(HAVE_TR1_UNORDERED_MAP_IN_TR1_NAMESPACE)
else (UNORDERED_MAP_FOUND)
  message("-- Replacing unordered_map/set with map/set (warning: slower!), "
    "try enabling CXX11 option if you expect C++11 to be available.")
  list(APPEND CERES_COMPILE_OPTIONS CERES_NO_UNORDERED_MAP)
endif()

include(FindSharedPtr)
find_shared_ptr()
if (SHARED_PTR_FOUND)
  if (SHARED_PTR_TR1_MEMORY_HEADER)
    list(APPEND CERES_COMPILE_OPTIONS CERES_TR1_MEMORY_HEADER)
  endif (SHARED_PTR_TR1_MEMORY_HEADER)
  if (SHARED_PTR_TR1_NAMESPACE)
    list(APPEND CERES_COMPILE_OPTIONS CERES_TR1_SHARED_PTR)
  endif (SHARED_PTR_TR1_NAMESPACE)
else (SHARED_PTR_FOUND)
  message(FATAL_ERROR "Unable to find shared_ptr, try enabling CXX11 option "
    "if you expect C++11 to be available.")
endif (SHARED_PTR_FOUND)

include(FindCXX11MathFunctions)
find_cxx11_math_functions()
if (CXX11 AND NOT CXX11_MATH_FUNCTIONS_FOUND)
  message("-- Failed to find C++11 math functions (cbrt(), exp2() etc). "
    "Disabling C++11.")
  update_cache_variable(CXX11 OFF)
endif()

if (CXX11)
  if (NOT HAVE_SHARED_PTR_IN_STD_NAMESPACE AND
      NOT HAVE_UNORDERED_MAP_IN_STD_NAMESPACE)
    message("-- Failed to find C++11 components in C++11 locations & "
      "namespaces, disabling CXX11.")
    update_cache_variable(CXX11 OFF)
  else()
    message("   ==============================================================")
    message("   Compiling Ceres using C++11.  This will result in a version ")
    message("   of Ceres that will require the use of C++11 in client code.")
    message("   ==============================================================")
    list(APPEND CERES_COMPILE_OPTIONS CERES_USE_CXX11)
    list(APPEND CERES_COMPILE_OPTIONS CERES_USE_CXX11_THREADS)
    if (COMPILER_HAS_CXX11_FLAG AND
        CMAKE_VERSION VERSION_LESS "2.8.12")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    endif()
  endif()
endif(CXX11)

if(UNIX)
list(APPEND CERES_COMPILE_OPTIONS CERES_HAVE_PTHREAD)
endif()

include_directories(
  include
  internal
  internal/ceres
  ${GLOG_INCLUDE_DIRS})

SET(EIGEN_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../eigen/" )

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

SET(SUITESPARSE_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../suitesparse/CHOLMOD/Include/")
LIST(APPEND SUITESPARSE_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../suitesparse/CCOLAMD/Include")
LIST(APPEND SUITESPARSE_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../suitesparse/CAMD/Include")
LIST(APPEND SUITESPARSE_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../suitesparse/COLAMD/Include")
LIST(APPEND SUITESPARSE_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../suitesparse/AMD/Include")
LIST(APPEND SUITESPARSE_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../suitesparse/metis-5.1.0/include")
LIST(APPEND SUITESPARSE_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../suitesparse/SuiteSparse_config/")
LIST(APPEND SUITESPARSE_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../suitesparse/SPQR/Include")
add_definitions("-DCERES_SUITESPARSE_VERSION=\"5.7.1\"")

include_directories(${SUITESPARSE_INCLUDE_DIRS})

if (ACCELERATESPARSE)
  find_package(AccelerateSparse)
  if (AccelerateSparse_FOUND)
    message("-- Found Apple's Accelerate framework with sparse solvers, "
      "building with Accelerate sparse support.")
  else()
    message("-- Failed to find Apple's Accelerate framework with sparse solvers, "
      "building without Accelerate sparse support.")
    update_cache_variable(ACCELERATESPARSE OFF)
    list(APPEND CERES_COMPILE_OPTIONS CERES_NO_ACCELERATE_SPARSE)
  endif()
else()
  message("-- Building without Apple's Accelerate sparse support.")
  list(APPEND CERES_COMPILE_OPTIONS CERES_NO_ACCELERATE_SPARSE)
  mark_as_advanced(FORCE AccelerateSparse_INCLUDE_DIR
                         AccelerateSparse_LIBRARY)
endif()

set (CERES_CXX_FLAGS)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
  if (CMAKE_COMPILER_IS_GNUCXX)
    # Linux
    if (CMAKE_SYSTEM_NAME MATCHES "Linux")
      if (NOT GCC_VERSION VERSION_LESS 4.2)
        set (CERES_CXX_FLAGS "${CERES_CXX_FLAGS} -march=native -mtune=native")
      endif (NOT GCC_VERSION VERSION_LESS 4.2)
    endif (CMAKE_SYSTEM_NAME MATCHES "Linux")
    # Mac OS X
    if (CMAKE_SYSTEM_NAME MATCHES "Darwin")
      set (CERES_CXX_FLAGS "${CERES_CXX_FLAGS} -msse3")
      # Use of -fast only applicable for Apple's GCC
      # Assume this is being used if GCC version < 4.3 on OSX
      execute_process(COMMAND ${CMAKE_C_COMPILER}
        ARGS ${CMAKE_CXX_COMPILER_ARG1} -dumpversion
        OUTPUT_VARIABLE GCC_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE)
      if (GCC_VERSION VERSION_LESS 4.3)
        set (CERES_CXX_FLAGS "${CERES_CXX_FLAGS} -fast")
      endif (GCC_VERSION VERSION_LESS 4.3)
    endif (CMAKE_SYSTEM_NAME MATCHES "Darwin")
  endif (CMAKE_COMPILER_IS_GNUCXX)
endif (CMAKE_BUILD_TYPE STREQUAL "Release")

set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${CERES_CXX_FLAGS}")

if (MSVC)
  # On MSVC, math constants are not included in <cmath> or <math.h> unless
  # _USE_MATH_DEFINES is defined [1].  As we use M_PI in the examples, ensure
  # that _USE_MATH_DEFINES is defined before the first inclusion of <cmath>.
  #
  # [1] https://msdn.microsoft.com/en-us/library/4hwaceh6.aspx
  add_definitions("-D_USE_MATH_DEFINES")
  # Disable signed/unsigned int conversion warnings.
  add_definitions("/wd4018")
  # Disable warning about using struct/class for the same symobl.
  add_definitions("/wd4099")
  # Disable warning about the insecurity of using "std::copy".
  add_definitions("/wd4996")
  # Disable performance warning about int-to-bool conversion.
  add_definitions("/wd4800")
  # Disable performance warning about fopen insecurity.
  add_definitions("/wd4996")
  # Disable warning about int64 to int32 conversion. Disabling
  # this warning may not be correct; needs investigation.
  # TODO(keir): Investigate these warnings in more detail.
  add_definitions("/wd4244")
  # It's not possible to use STL types in DLL interfaces in a portable and
  # reliable way. However, that's what happens with Google Log and Google Flags
  # on Windows. MSVC gets upset about this and throws warnings that we can't do
  # much about. The real solution is to link static versions of Google Log and
  # Google Test, but that seems tricky on Windows. So, disable the warning.
  add_definitions("/wd4251")

  # Google Flags doesn't have their DLL import/export stuff set up correctly,
  # which results in linker warnings. This is irrelevant for Ceres, so ignore
  # the warnings.
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /ignore:4049")

  # Update the C/CXX flags for MSVC to use either the static or shared
  # C-Run Time (CRT) library based on the user option: MSVC_USE_STATIC_CRT.
  list(APPEND C_CXX_FLAGS
    CMAKE_CXX_FLAGS
    CMAKE_CXX_FLAGS_DEBUG
    CMAKE_CXX_FLAGS_RELEASE
    CMAKE_CXX_FLAGS_MINSIZEREL
    CMAKE_CXX_FLAGS_RELWITHDEBINFO)

  foreach(FLAG_VAR ${C_CXX_FLAGS})
    if (MSVC_USE_STATIC_CRT)
      # Use static CRT.
      if (${FLAG_VAR} MATCHES "/MD")
        string(REGEX REPLACE "/MD" "/MT" ${FLAG_VAR} "${${FLAG_VAR}}")
      endif (${FLAG_VAR} MATCHES "/MD")
    else (MSVC_USE_STATIC_CRT)
      # Use shared, not static, CRT.
      if (${FLAG_VAR} MATCHES "/MT")
        string(REGEX REPLACE "/MT" "/MD" ${FLAG_VAR} "${${FLAG_VAR}}")
      endif (${FLAG_VAR} MATCHES "/MT")
    endif (MSVC_USE_STATIC_CRT)
  endforeach()

  # Tuple sizes of 10 are used by Gtest.
  add_definitions("-D_VARIADIC_MAX=10")

  include(CheckIfUnderscorePrefixedBesselFunctionsExist)
  check_if_underscore_prefixed_bessel_functions_exist(
    HAVE_UNDERSCORE_PREFIXED_BESSEL_FUNCTIONS)
  if (HAVE_UNDERSCORE_PREFIXED_BESSEL_FUNCTIONS)
    list(APPEND CERES_COMPILE_OPTIONS
      CERES_MSVC_USE_UNDERSCORE_PREFIXED_BESSEL_FUNCTIONS)
  endif()
endif (MSVC)

if (UNIX)
  # GCC is not strict enough by default, so enable most of the warnings.
  set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Wno-unknown-pragmas -Wno-sign-compare -Wno-unused-parameter -Wno-missing-field-initializers")
endif (UNIX)

# Use a larger inlining threshold for Clang, since it hobbles Eigen,
# resulting in an unreasonably slow version of the blas routines. The
# -Qunused-arguments is needed because CMake passes the inline
# threshold to the linker and clang complains about it and dies.
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Qunused-arguments -mllvm -inline-threshold=600")

  # Older versions of Clang (<= 2.9) do not support the 'return-type-c-linkage'
  # option, so check for its presence before adding it to the default flags set.
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag("-Wno-return-type-c-linkage"
                          HAVE_RETURN_TYPE_C_LINKAGE)
  if (HAVE_RETURN_TYPE_C_LINKAGE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-return-type-c-linkage")
  endif(HAVE_RETURN_TYPE_C_LINKAGE)
endif ()

# Xcode 4.5.x used Clang 4.1 (Apple version), this has a bug that prevents
# compilation of Ceres.
if (APPLE AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  execute_process(COMMAND ${CMAKE_CXX_COMPILER}
    ARGS ${CMAKE_CXX_COMPILER_ARG1} -dumpversion
    OUTPUT_VARIABLE CLANG_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  # Use version > 4.0 & < 4.2 to catch all 4.1(.x) versions.
  if (CLANG_VERSION VERSION_GREATER 4.0 AND
      CLANG_VERSION VERSION_LESS 4.2)
    message(FATAL_ERROR "You are attempting to build Ceres on OS X using Xcode "
      "4.5.x (Clang version: ${CLANG_VERSION}). This version of Clang has a "
      "bug that prevents compilation of Ceres, please update to "
      "Xcode >= 4.6.3.")
  endif (CLANG_VERSION VERSION_GREATER 4.0 AND
    CLANG_VERSION VERSION_LESS 4.2)
endif (APPLE AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang")

list(REMOVE_DUPLICATES CERES_COMPILE_OPTIONS)
include(CreateCeresConfig)
create_ceres_config("${CERES_COMPILE_OPTIONS}"
  ${Ceres_BINARY_DIR}/config/ceres/internal)
include_directories(BEFORE ${Ceres_BINARY_DIR}/config)

add_subdirectory(internal/ceres)
