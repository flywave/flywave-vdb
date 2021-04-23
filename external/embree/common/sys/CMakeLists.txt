## Copyright 2009-2020 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

SET(CMAKE_THREAD_PREFER_PTHREAD TRUE)
FIND_PACKAGE(Threads REQUIRED)

ADD_LIBRARY(sys STATIC
  sysinfo.cpp
  alloc.cpp
  filename.cpp
  library.cpp
  thread.cpp
  string.cpp
  regression.cpp
  mutex.cpp
  condition.cpp
  barrier.cpp
)

TARGET_LINK_LIBRARIES(sys ${CMAKE_THREAD_LIBS_INIT} ${CMAKE_DL_LIBS})
SET_PROPERTY(TARGET sys PROPERTY FOLDER external/embree/common)
SET_PROPERTY(TARGET sys APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
