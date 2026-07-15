# SPDX-License-Identifier: BSD-3-Clause
# Copyright Contributors to the OpenEXR Project.

#
# Imath
#

set(IMATH_LIBRARY Imath)

set(IMATH_SOURCES
    half.cpp
    ImathColorAlgo.cpp
    ImathFun.cpp
    ImathMatrixAlgo.cpp
    ImathRandom.cpp
    toFloat.h
)

set(IMATH_HEADERS
    half.h
    halfFunction.h
    halfLimits.h
    ImathBox.h
    ImathBoxAlgo.h
    ImathColor.h
    ImathColorAlgo.h
    ImathEuler.h
    ImathExport.h
    ImathForward.h
    ImathFrame.h
    ImathFrustum.h
    ImathFrustumTest.h
    ImathFun.h
    ImathGL.h
    ImathGLU.h
    ImathInt64.h
    ImathInterval.h
    ImathLine.h
    ImathLineAlgo.h
    ImathMath.h
    ImathMatrix.h
    ImathMatrixAlgo.h
    ImathNamespace.h
    ImathPlane.h
    ImathPlatform.h
    ImathQuat.h
    ImathRandom.h
    ImathRoots.h
    ImathShear.h
    ImathSphere.h
    ImathTypeTraits.h
    ImathVec.h
    ImathVecAlgo.h
    ${IMATH_CONFIG_DIR}/ImathConfig.h
)

add_library(${IMATH_LIBRARY} STATIC ${IMATH_HEADERS} ${IMATH_SOURCES})


add_library(Imath::Imath ALIAS ${IMATH_LIBRARY})

#
# Compile options
#

target_compile_features(${IMATH_LIBRARY}
    PRIVATE cxx_std_${IMATH_CXX_STANDARD}
    INTERFACE cxx_std_11
)

set_target_properties(${IMATH_LIBRARY} PROPERTIES
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
)

if (NOT IMATH_USE_DEFAULT_VISIBILITY)
    set_target_properties(${IMATH_LIBRARY} PROPERTIES
      C_VISIBILITY_PRESET hidden
      CXX_VISIBILITY_PRESET hidden
      VISIBILITY_INLINES_HIDDEN ON
    )
else()
    target_compile_definitions(${IMATH_LIBRARY} PUBLIC IMATH_USE_DEFAULT_VISIBILITY)
endif()

if (MSVC)
    target_compile_options(${IMATH_LIBRARY} PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:/EHsc>")
endif()

set_property(TARGET ${IMATH_LIBRARY} PROPERTY PUBLIC_HEADER ${IMATH_HEADERS})

target_include_directories(${IMATH_LIBRARY} INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

#
# Link options
#
target_link_libraries(${IMATH_LIBRARY} PUBLIC ImathConfig)

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(${IMATH_LIBRARY} PROPERTIES FOLDER external)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)

SET_TARGET_PROPERTIES(${IMATH_LIBRARY} PROPERTIES
ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}
ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR})
SET_TARGET_PROPERTIES(${IMATH_LIBRARY} PROPERTIES 
LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}
LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR})
SET_TARGET_PROPERTIES(${IMATH_LIBRARY} PROPERTIES
RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}
RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR})
