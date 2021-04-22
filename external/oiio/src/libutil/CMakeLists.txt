set (libOpenImageIO_Util_srcs argparse.cpp benchmark.cpp
                  errorhandler.cpp filesystem.cpp
                  farmhash.cpp filter.cpp hashes.cpp paramlist.cpp
                  plugin.cpp SHA1.cpp
                  strutil.cpp sysutil.cpp thread.cpp timer.cpp
                  typedesc.cpp ustring.cpp xxhash.cpp)

add_library (OpenImageIO_Util STATIC ${libOpenImageIO_Util_srcs})
target_include_directories (OpenImageIO_Util
        PUBLIC
            ${CMAKE_INSTALL_FULL_INCLUDEDIR}
            ${IMATH_INCLUDES} ${OPENEXR_INCLUDES}
        )

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(OpenImageIO_Util PROPERTIES FOLDER external/oiio)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
