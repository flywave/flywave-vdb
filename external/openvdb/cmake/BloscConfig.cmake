set(Blosc_VERSION 1.21.0)
set(Blosc_FOUND TRUE)
if(NOT TARGET Blosc::blosc)
  add_library(Blosc::blosc STATIC IMPORTED)
  set_target_properties(Blosc::blosc PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/../../../lib/libblosc.a"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/../../c-blosc"
  )
endif()
