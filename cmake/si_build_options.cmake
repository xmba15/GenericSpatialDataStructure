option(BUILD_EXAMPLES "Option whether to build examples" OFF)

if(${BUILD_EXAMPLES})
  message(STATUS "Also build examples")
  add_subdirectory(examples)
endif(${BUILD_EXAMPLES})

option(WITH_DEBUG "Enable debug" OFF)
if(WITH_DEBUG)
  target_compile_definitions(${PROJECT_TARGET_LIB_NAME}
    PUBLIC
      WITH_DEBUG
  )
endif(WITH_DEBUG)

option(WITH_GTEST "Enable test" OFF)
if(WITH_GTEST)
  enable_testing()
  add_subdirectory(tests)
endif(WITH_GTEST)
