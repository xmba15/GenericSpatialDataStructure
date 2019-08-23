list(APPEND EXAMPLES
  "ExQuadtree"
  "ExOctree"
  "ExSpaceFillingCurve"
  "ExBitTwiddling"
)

include(example_utility)

list(APPEND PUBLIC_LIBS
  ${PROJECT_TARGET_LIB_NAME}
)

list(APPEND PRIVATE_LIBS
)

foreach(EXAMPLE ${EXAMPLES})
  __build_example(
    ${EXAMPLE}
    "${PUBLIC_LIBS}"
    "${PRIVATE_LIBS}"
  )
endforeach(EXAMPLE)
