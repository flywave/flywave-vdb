set(TBB_VERSION 2021.1)
set(TBB_FOUND TRUE)
if(NOT TARGET TBB::tbb)
  add_library(TBB::tbb STATIC IMPORTED)
  set_target_properties(TBB::tbb PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/../../../lib/libtbb.a"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/../../tbb/include"
  )
endif()
