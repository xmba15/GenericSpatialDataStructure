cmake_minimum_required(VERSION 3.8)

project(libspatial_indexing)

set(PROJECT_TARGET_LIB_NAME "spatial_indexing")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

add_subdirectory(src)

include(si_build_options)
