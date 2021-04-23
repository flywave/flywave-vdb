## Copyright 2009-2020 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

ADD_LIBRARY(lexers STATIC
 stringstream.cpp
 tokenstream.cpp
)
TARGET_LINK_LIBRARIES(lexers sys math)
SET_PROPERTY(TARGET lexers PROPERTY FOLDER external/embree/common)
SET_PROPERTY(TARGET lexers APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
