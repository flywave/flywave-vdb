## Copyright 2009-2020 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

ADD_LIBRARY(math STATIC constants.cpp)
SET_PROPERTY(TARGET math PROPERTY FOLDER external/embree/common)
SET_PROPERTY(TARGET math APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
