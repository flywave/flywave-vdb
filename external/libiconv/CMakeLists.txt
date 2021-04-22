cmake_minimum_required(VERSION 2.8.12)
project(iconv)

add_definitions(
  -DLIBICONV_PLUG
)

set(TARGET_NAME iconv)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/src" src)
