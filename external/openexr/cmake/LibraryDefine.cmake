# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) Contributors to the OpenEXR Project.

# NB: This function has a number of Imath-specific names/variables
# in it, so be careful copying...
function(OPENEXR_DEFINE_LIBRARY libname)
  set(options)
  set(oneValueArgs PRIV_EXPORT CURDIR CURBINDIR)
  set(multiValueArgs SOURCES HEADERS DEPENDENCIES PRIVATE_DEPS)
  cmake_parse_arguments(OPENEXR_CURLIB "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if (MSVC)
    set(_openexr_extra_flags "$<$<COMPILE_LANGUAGE:CXX>:/EHsc>" "$<$<COMPILE_LANGUAGE:CXX>:/MP>")
  endif()
  set(objlib ${libname})
  add_library(${objlib} STATIC 
    ${OPENEXR_CURLIB_HEADERS}
    ${OPENEXR_CURLIB_SOURCES})

  # Use ${OPENEXR_CXX_STANDARD} to determine the standard we use to compile
  # OpenEXR itself. The headers will use string_view and such, so ensure
  # the user is at least 17, but could be higher
  target_compile_features(${objlib}
                          PRIVATE cxx_std_${OPENEXR_CXX_STANDARD}
                          INTERFACE cxx_std_17 )

  if(OPENEXR_CURLIB_CURDIR)
    target_include_directories(${objlib} INTERFACE $<BUILD_INTERFACE:${OPENEXR_CURLIB_CURDIR}>)
  endif()
  if(OPENEXR_CURLIB_CURBINDIR)
    target_include_directories(${objlib} PRIVATE $<BUILD_INTERFACE:${OPENEXR_CURLIB_CURBINDIR}>)
  endif()

  set_target_properties(${objlib} PROPERTIES
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
  )
  if (NOT OPENEXR_USE_DEFAULT_VISIBILITY)
    set_target_properties(${objlib} PROPERTIES
      C_VISIBILITY_PRESET hidden
      CXX_VISIBILITY_PRESET hidden
      VISIBILITY_INLINES_HIDDEN ON
      )
  else()
      target_compile_definitions(${objlib} PUBLIC OPENEXR_USE_DEFAULT_VISIBILITY)
  endif()
  if (_openexr_extra_flags)
    target_compile_options(${objlib} PRIVATE ${_openexr_extra_flags})
  endif()
  set_property(TARGET ${objlib} PROPERTY PUBLIC_HEADER ${OPENEXR_CURLIB_HEADERS})

  IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
    SET_TARGET_PROPERTIES(${libname} PROPERTIES FOLDER external/openexr)
  ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)

  SET_TARGET_PROPERTIES(${libname} PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/../
  ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/../)
  SET_TARGET_PROPERTIES(${libname} PROPERTIES 
  LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/../
  LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/../)
  SET_TARGET_PROPERTIES(${libname} PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/../
  RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/../)

endfunction()
