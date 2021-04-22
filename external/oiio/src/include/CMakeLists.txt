# Copyright 2008-present Contributors to the OpenImageIO project.
# SPDX-License-Identifier: BSD-3-Clause
# https://github.com/OpenImageIO/oiio/blob/master/LICENSE.md

file (GLOB public_headers OpenImageIO/*.h)

# Mangle the SOVERSION so that it's a valid C++ identifier for the
# versioning namespace defined in ${versionfile}
string (REGEX REPLACE "\\." "_" MANGLED_SOVERSION ${SOVERSION})
set (OIIO_VERSION_NS "v${MANGLED_SOVERSION}")

# Generate version.h
set (versionfile oiioversion.h)
if (VERBOSE)
    message(STATUS "Create ${versionfile} from ${versionfile}.in")
endif ()
configure_file (${PROJECT_NAME}/${versionfile}.in "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/${versionfile}" @ONLY)
list (APPEND public_headers "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/${versionfile}")

# Generate Imath.h
if (VERBOSE)
    message(STATUS "Create Imath.h from Imath.h.in")
endif ()
configure_file (${PROJECT_NAME}/Imath.h.in "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/Imath.h" @ONLY)
list (APPEND public_headers "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/Imath.h")
