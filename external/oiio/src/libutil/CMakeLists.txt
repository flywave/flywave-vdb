# Copyright Contributors to the OpenImageIO project.
# SPDX-License-Identifier: Apache-2.0
# https://github.com/AcademySoftwareFoundation/OpenImageIO

set (libOpenImageIO_Util_srcs argparse.cpp benchmark.cpp
                  errorhandler.cpp farmhash.cpp filesystem.cpp
                  fmath.cpp filter.cpp hashes.cpp paramlist.cpp
                  plugin.cpp SHA1.cpp
                  strutil.cpp sysutil.cpp thread.cpp timer.cpp
                  typedesc.cpp ustring.cpp xxhash.cpp)

if (CMAKE_COMPILER_IS_GNUCC AND NOT ${GCC_VERSION} VERSION_LESS 9.0)
    set_property (SOURCE SHA1.cpp
                  APPEND PROPERTY COMPILE_OPTIONS -Wno-stringop-truncation)
endif ()

if (CMAKE_UNITY_BUILD)
    # If unity builds are occurring, fmath.cpp MUST be compiled alone. But the
    # rest can be built together if in group mode.
    set_property (SOURCE fmath.cpp APPEND PROPERTY SKIP_UNITY_BUILD_INCLUSION TRUE)
    set_source_files_properties (${libOpenImageIO_Util_srcs}
                                 PROPERTIES UNITY_GROUP utilsrc)
endif ()


set (OpenImageIO_Util_LINK_FLAGS "${VISIBILITY_MAP_COMMAND} ${EXTRA_DSO_LINK_ARGS}")
if (UNIX AND NOT APPLE)
    # Hide symbols from any static dependent libraries embedded here.
    set (OpenImageIO_Util_LINK_FLAGS "${OpenImageIO_Util_LINK_FLAGS} -Wl,--exclude-libs,ALL")
endif ()


function (setup_oiio_util_library targetname)
    if (WIN32)
        configure_file(../build-scripts/version_win32.rc.in "${CMAKE_CURRENT_BINARY_DIR}/version_win32.rc" @ONLY)
        add_library (${targetname} STATIC ${libOpenImageIO_Util_srcs} ${CMAKE_CURRENT_BINARY_DIR}/version_win32.rc)
    else ()
        add_library (${targetname} STATIC ${libOpenImageIO_Util_srcs})
    endif ()

    target_compile_definitions(${targetname} PRIVATE
                               ${${PROJECT_NAME}_compile_definitions})
    target_compile_options(${targetname} PRIVATE
                               ${${PROJECT_NAME}_compile_options})
    target_link_options(${targetname} PRIVATE
                               ${${PROJECT_NAME}_link_options})

    target_include_directories (${targetname}
            PUBLIC
                $<BUILD_INTERFACE:${OpenImageIO_LOCAL_DEPS_ROOT}/include>
                $<INSTALL_INTERFACE:include>
            PRIVATE
                ${ROBINMAP_INCLUDES}
            )
 
    if (GCC_VERSION VERSION_GREATER_EQUAL 12.0
          AND GCC_VERSION VERSION_LESS 14.0
          AND CMAKE_CXX_STANDARD VERSION_GREATER_EQUAL 23)
        target_link_libraries (${targetname} PRIVATE stdc++_libbacktrace)
    elseif (GCC_VERSION VERSION_GREATER_EQUAL 14.0
            AND CMAKE_CXX_STANDARD VERSION_GREATER_EQUAL 23)
        target_link_libraries (${targetname} PRIVATE stdc++exp)
    endif ()

    if (WIN32)
        target_compile_definitions(${targetname} PRIVATE
                                   WIN32_LEAN_AND_MEAN NOMINMAX
                                   NOGDI VC_EXTRALEAN)
        target_link_libraries (${targetname} PRIVATE psapi)
    endif()

    target_compile_definitions (${targetname} PRIVATE OpenImageIO_EXPORTS)


    if (NOT BUILD_SHARED_LIBS OR targetname MATCHES "static")
        target_compile_definitions (${targetname} PUBLIC OIIO_STATIC_DEFINE=1)
    endif ()

    # Propagate C++ minimum to downstream consumers
    target_compile_features (${targetname}
                             INTERFACE cxx_std_${DOWNSTREAM_CXX_STANDARD})

    set_target_properties(${targetname}
                          PROPERTIES
                             VERSION     ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}
                             SOVERSION   ${SOVERSION}
                             OUTPUT_NAME ${targetname}${OIIO_LIBNAME_SUFFIX}
                             POSITION_INDEPENDENT_CODE ON
                         )

    set_target_properties (${targetname} PROPERTIES
                           LINK_FLAGS ${OpenImageIO_Util_LINK_FLAGS})

    IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
    SET_TARGET_PROPERTIES(${targetname} PROPERTIES FOLDER external/oiio)
    ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)

    SET_TARGET_PROPERTIES(${targetname} PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/../../
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/../../)
    SET_TARGET_PROPERTIES(${targetname} PROPERTIES 
    LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/../../
    LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/../../)
    SET_TARGET_PROPERTIES(${targetname} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/../../
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/../../)

endfunction()


setup_oiio_util_library (OpenImageIO_Util)
