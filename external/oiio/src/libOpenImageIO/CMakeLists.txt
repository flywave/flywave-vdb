# Copyright Contributors to the OpenImageIO project.
# SPDX-License-Identifier: Apache-2.0
# https://github.com/AcademySoftwareFoundation/OpenImageIO

if (VERBOSE)
   message (STATUS "Create buildopts.h from buildopts.h.in")
endif ()
configure_file (buildopts.h.in "${CMAKE_CURRENT_BINARY_DIR}/../../include/buildopts.h" @ONLY)

file (GLOB libOpenImageIO_hdrs ../include/OpenImageIO/*.h)

if (NOT USE_EXTERNAL_PUGIXML)
    list (APPEND libOpenImageIO_hdrs
          ../include/OpenImageIO/detail/pugixml/pugiconfig.hpp
          ../include/OpenImageIO/detail/pugixml/pugixml.hpp
          ../include/OpenImageIO/detail/pugixml/pugixml.cpp
    )
    if (CMAKE_COMPILER_IS_GNUCC AND NOT ${GCC_VERSION} VERSION_LESS 6.0)
        set_source_files_properties (formatspec.cpp xmp.cpp
                                     PROPERTIES COMPILE_FLAGS -Wno-error=placement-new)
    endif ()
endif()

# Make the build complete for newer ffmpeg versions (3.1.1+) that have
# marked m_format_context->streams[i]->codec as deprecated.
# FIXME -- at some point, come back and figure out how to fix for real
# before the field disappears entirely.
if (NOT MSVC)
    set_source_files_properties (../ffmpeg.imageio/ffmpeginput.cpp
                             PROPERTIES COMPILE_FLAGS "-Wno-deprecated-declarations")
endif()

if (CMAKE_COMPILER_IS_GNUCC AND ${GCC_VERSION} VERSION_GREATER_EQUAL 9.0)
    set_source_files_properties (../libutil/SHA1.cpp
                                 PROPERTIES COMPILE_FLAGS -Wno-stringop-truncation)
endif ()

if (CMAKE_COMPILER_IS_GNUCC AND ${GCC_VERSION} VERSION_GREATER_EQUAL 6.0
    AND ${GCC_VERSION} VERSION_LESS 7.0)
    set_source_files_properties (../openvdb.imageio/openvdbinput.cpp
                                 PROPERTIES COMPILE_FLAGS -Wno-error=strict-overflow)
endif ()

set (libOpenImageIO_srcs
                          imagebufalgo.cpp
                          imagebufalgo_pixelmath.cpp
                          imagebufalgo_channels.cpp
                          imagebufalgo_compare.cpp
                          imagebufalgo_copy.cpp
                          imagebufalgo_deep.cpp
                          imagebufalgo_draw.cpp
                          imagebufalgo_addsub.cpp
                          imagebufalgo_muldiv.cpp
                          imagebufalgo_mad.cpp
                          imagebufalgo_minmaxchan.cpp
                          imagebufalgo_orient.cpp
                          imagebufalgo_xform.cpp
                          imagebufalgo_demosaic.cpp
                          imagebufalgo_yee.cpp
                          imagebufalgo_yee.cpp
                          deepdata.cpp exif.cpp exif-canon.cpp
                          formatspec.cpp
                          icc.cpp imagebuf.cpp
                          imageinput.cpp imageio.cpp imageioplugin.cpp
                          imageoutput.cpp
                          iptc.cpp xmp.cpp
                          color_ocio.cpp
                          maketexture.cpp
                          bluenoise.cpp
                          printinfo.cpp
                          oiio_gpu.cpp
                          ../libtexture/texturesys.cpp
                          ../libtexture/texture3d.cpp
                          ../libtexture/environment.cpp
                          ../libtexture/texoptions.cpp
                          ../libtexture/imagecache.cpp
                          ${libOpenImageIO_srcs}
                          ${libOpenImageIO_hdrs}
                         )

add_library (OpenImageIO STATIC ${libOpenImageIO_srcs})

# If the 'EMBEDPLUGINS' option is set, we want to compile the source for
# all the plugins into libOpenImageIO.
if (EMBEDPLUGINS)
    target_compile_definitions (OpenImageIO
                                PRIVATE
                                    EMBED_PLUGINS=1
                                    ${format_plugin_definitions})
    target_compile_options (OpenImageIO
                            PRIVATE ${format_plugin_compile_options})
    target_include_directories (OpenImageIO BEFORE
                                PRIVATE ${format_plugin_include_dirs})

    # Organize the embedded plugins into source folders
    set (plugin_types "")
    foreach (src ${libOpenImageIO_srcs})
        if (src MATCHES "^.+/([^/]+)\\.imageio/.+$")
            set (plugin_types ${plugin_types} ${CMAKE_MATCH_1})
        endif ()
    endforeach ()
    list (REMOVE_DUPLICATES plugin_types)
    foreach (plugin ${plugin_types})
        source_group ("Plugins\\${plugin}"
                      REGULAR_EXPRESSION "^.+/${plugin}\\.imageio/.+$"
                     )
    endforeach ()
endif ()

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(OpenImageIO PROPERTIES FOLDER external/oiio)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)

SET_TARGET_PROPERTIES(OpenImageIO PROPERTIES
ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/../../
ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/../../)
SET_TARGET_PROPERTIES(OpenImageIO PROPERTIES 
LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/../../
LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/../../)
SET_TARGET_PROPERTIES(OpenImageIO PROPERTIES
RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/../../
RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/../../)
