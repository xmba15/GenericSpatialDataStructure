include(matplotlibcpp)
__fetch_matplotlibcpp(
  "${PROJECT_SOURCE_DIR}/cmake"
  "${PROJECT_BINARY_DIR}/${PROJECT_TARGET_LIB_NAME}_matplotlibcpp"
)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

list(APPEND EXAMPLES
  "ExQuadtree"
  "ExOctree"
  "ExSpaceFillingCurve"
  "ExBitTwiddling"
  "ExPCDIO"
  "ExOctreeOpenGL"
)

include(example_utility)

add_definitions(-DDATA_PATH="${PROJECT_SOURCE_DIR}/data")

list(APPEND PUBLIC_LIBS
  ${PROJECT_TARGET_LIB_NAME}
  matplotlib_cpp
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARY}
)

list(APPEND PRIVATE_LIBS
)

foreach(EXAMPLE ${EXAMPLES})
  __build_example(
    ${EXAMPLE}
    "${PUBLIC_LIBS}"
    "${PRIVATE_LIBS}"
  )
  target_include_directories(${EXAMPLE}
    PUBLIC
      ${OPENGL_INCLUDE_DIRS}
      ${GLUT_INCLUDE_DIRS}
  )
endforeach(EXAMPLE)
