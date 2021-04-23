## Copyright 2009-2020 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

ADD_LIBRARY(simd STATIC sse.cpp)
SET_PROPERTY(TARGET simd PROPERTY FOLDER external/embree/common)
SET_PROPERTY(TARGET simd APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
