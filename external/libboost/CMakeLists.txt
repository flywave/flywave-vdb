# Build Boost 1.88 via its native Boost-CMake. The hand-maintained per-lib .cpp
# lists drift across Boost versions (libs became header-only / restructured), so
# we delegate to Boost's own CMake which knows the correct sources per library.
set(BOOST_INCLUDE_LIBRARIES "chrono;date_time;filesystem;iostreams;program_options;serialization;system;thread;timer" CACHE STRING "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BOOST_SKIP_INSTALL_RULES ON CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BOOST_ALL_DYN_LINK OFF CACHE BOOL "" FORCE)
add_definitions(-DBOOST_ALL_NO_LIB -DBOOST_STATIC_LINK)

# Collect all static boost archives into one directory so the link line can use -L.
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/external/libboost/lib")

add_subdirectory(boost_1_88_0)
