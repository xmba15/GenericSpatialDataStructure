project(liblzf)
file(GLOB LIBLZF_SOURCE_FILES "${PROJECT_SOURCE_DIR}/liblzf/*.c")

add_library(liblzf STATIC ${LIBLZF_SOURCE_FILES})
