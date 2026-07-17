set(Imath_VERSION 3.2.2)
set(Imath_FOUND TRUE)
if(NOT TARGET Imath::Imath)
  add_library(Imath::Imath STATIC IMPORTED)
  set_target_properties(Imath::Imath PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/../../../lib/libImath.a"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/../../imath/src;${CMAKE_CURRENT_LIST_DIR}/../../imath"
  )
endif()
