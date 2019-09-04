cmake_minimum_required(VERSION 3.8)

project(libspatial_partioning)

set(PROJECT_TARGET_LIB_NAME "spatial_partioning")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/liblzf)
add_subdirectory(src)

include(si_build_options)
